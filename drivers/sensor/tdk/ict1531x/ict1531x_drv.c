/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/logging/log.h>

#include "ict1531x_drv.h"

LOG_MODULE_REGISTER(ict1531x, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error "ict1531x driver enabled without any devices"
#endif

#define ICT1531X_ALL_MAGN_CHAN(x) (((x) == (SENSOR_CHAN_MAGN_XYZ)) \
                                  || ((x) == (SENSOR_CHAN_MAGN_X)) \
                                  || ((x) == (SENSOR_CHAN_MAGN_Y)) \
                                  || ((x) == (SENSOR_CHAN_MAGN_Z)))

#define ICT1531X_MEASUREMENT_TIME_US 3050*2
#define ICT1531X_MAGN_VALUE_nT(val) ((val) * 75)
#define ICT1531X_TEMPERATURE_VALUE_mC(val) (25000 + ((val)*625 / 100))

uint64_t inv_ict1531x_get_time_us(void)
{
  uint64_t time_ms = k_uptime_get();
  uint32_t cycles = sys_clock_tick_get_32();
  if(time_ms != k_uptime_get())
  {
    // ms tick increase while getting cycle count => get new values
    time_ms = k_uptime_get();
    cycles = sys_clock_tick_get_32();
  }

  return (time_ms * 1000) + (((uint64_t) cycles * 1000000) / (sys_clock_hw_cycles_per_sec()))%1000;
}

static void ict1531x_convert_mag(struct sensor_value *val, int16_t raw_val)
{
	int32_t conv_val;

  /* ICT1531x magnetometer sensitivity is 75nT/LSB */
	conv_val = ((int32_t)raw_val * 75);
  
  /* Magnetic field is expressed in Gauss 1Gs = 10^-4 Tesla = 10^5 nT */
	val->val1 = conv_val / 100000;
	val->val2 = (conv_val % 100000) * 10;
}

static void ict1531x_convert_temperature(struct sensor_value *val, int16_t raw_val)
{
	int32_t conv_val;

  /* ICT1531x temperature sensitivity is 6.25mdegC/LSB */
	conv_val = 25000 + (((int32_t)raw_val * 625) / 100);
  
  /* Temperature is expressed in degree C */
	val->val1 = conv_val / 1000;
	val->val2 = (conv_val % 1000) * 1000;
}

static int inv_io_hal_read_reg(void *ctx, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
	struct device *dev = (struct device *)ctx;
	const struct ict1531x_config *cfg = (const struct ict1531x_config *)dev->config;

	return i2c_burst_read_dt(&cfg->i2c, reg, (uint8_t *)rbuffer, rlen);
}

static int inv_io_hal_write_reg(void *ctx, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
	struct device *dev = (struct device *)ctx;
	const struct ict1531x_config *cfg = (const struct ict1531x_config *)dev->config;

	return i2c_burst_write_dt(&cfg->i2c, reg, (uint8_t *)wbuffer, wlen);
}

static int ict1531x_sample_fetch(const struct device *dev, const enum sensor_channel chan)
{
	struct ict1531x_data *data = (struct ict1531x_data *)dev->data;
	int rc = 0;
  // Max measurement time is 3355us < (ICT1531X_MEASUREMENT_TIME_US + 6*51 us)
	int8_t trials = 6; 

	if (!(ICT1531X_ALL_MAGN_CHAN(chan) || (chan == SENSOR_CHAN_AMBIENT_TEMP)
    || (chan == SENSOR_CHAN_ALL))) {
		return -ENOTSUP;
	}
	rc = inv_ict1531x_enable_sensor(&data->ict_device, 1);

	/* Initial sleep waiting the sensor proceeds with the measure = 3050us */
	k_sleep(K_USEC(ICT1531X_MEASUREMENT_TIME_US));
	do {
		k_sleep(K_USEC(51));
		rc = inv_ict1531x_poll_data(&data->ict_device, data->raw_mag, &(data->raw_temperature));
	} while ((rc == 1) && (trials-- > 0));

  /* Go back to sleep mode */
  rc |= inv_ict1531x_enable_sensor(&data->ict_device, 0);
  return rc;
}

static int ict1531x_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct ict1531x_data *data = (struct ict1531x_data *)dev->data;

	if (!(ICT1531X_ALL_MAGN_CHAN(chan) || (chan == SENSOR_CHAN_AMBIENT_TEMP))) {
		return -ENOTSUP;
	}

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		ict1531x_convert_temperature(val, data->raw_temperature);
	} else if (chan == SENSOR_CHAN_MAGN_XYZ) {
    for (int i = 0 ; i < 3 ; i++)
    {
      ict1531x_convert_mag(&val[i],data->raw_mag[i]);
    }
	} else if (chan == SENSOR_CHAN_MAGN_X) {
      ict1531x_convert_mag(val,data->raw_mag[0]);
	} else if (chan == SENSOR_CHAN_MAGN_X) {
      ict1531x_convert_mag(val,data->raw_mag[1]);
	} else if (chan == SENSOR_CHAN_MAGN_X) {
      ict1531x_convert_mag(val,data->raw_mag[2]);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int ict1531x_init(const struct device *dev)
{
	struct inv_ict1531x_serif ict1531x_serif;
	int rc = 0;
	struct ict1531x_data *data = (struct ict1531x_data *)dev->data;
  uint8_t who_am_i = 0;

	ict1531x_serif.context = (void *)dev;
	ict1531x_serif.read_reg = inv_io_hal_read_reg;
	ict1531x_serif.write_reg = inv_io_hal_write_reg;
	ict1531x_serif.max_read = 2048;  /* maximum number of bytes allowed per serial read */
	ict1531x_serif.max_write = 2048; /* maximum number of bytes allowed per serial write */
	ict1531x_serif.is_spi = 0;
	/*
	 * Reset Magnetometer driver states
	 */
	inv_ict1531x_reset_states(&(data->ict_device), &ict1531x_serif);

  rc = inv_ict1531x_get_whoami(&(data->ict_device), &who_am_i);
  if((rc != 0) ||(who_am_i != ICT1531X_WHOAMI))
  {
		LOG_ERR("Whoami error %d, read %02x, exp %02x", rc, who_am_i, ICT1531X_WHOAMI);
		return rc;
  }

	rc = inv_ict1531x_soft_reset(&data->ict_device);
	if (rc != 0) {
		LOG_ERR("Soft reset error %d", rc);
		return rc;
	}

	/* successful init, return 0 */
	return 0;
}

static const struct sensor_driver_api ict1531x_api_funcs = {
	.sample_fetch = ict1531x_sample_fetch,
	.channel_get = ict1531x_channel_get,
};

#define ICT1531X_DEFINE(inst)                                                                      \
	static struct ict1531x_data ict1531x_drv_##inst;                                                  \
	static const struct ict1531x_config ict1531x_config_##inst = {                             \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, ict1531x_init, NULL, &ict1531x_drv_##inst,                     \
			      &ict1531x_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,   \
			      &ict1531x_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(ICT1531X_DEFINE)
