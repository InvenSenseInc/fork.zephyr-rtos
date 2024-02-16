/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/logging/log.h>

#include "icp201xx.h"

LOG_MODULE_REGISTER(ICP201XX, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error "ICP201XX driver enabled without any devices"
#endif

void inv_icp201xx_sleep_us(int us)
{
	k_sleep(K_USEC(us));
}

#if ICP201XX_BUS_I2C
static int inv_io_hal_read_reg(void* ctx, 
				uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
	struct device* dev = (struct device*)ctx;
	const struct icp201xx_config* cfg = (const struct icp201xx_config*)dev->config;
	return i2c_burst_read_dt(&cfg->i2c, reg, (uint8_t*)rbuffer, rlen);
}

static int inv_io_hal_write_reg(void* ctx, uint8_t reg, 
                         const uint8_t *wbuffer, uint32_t wlen)
{
	struct device* dev = (struct device*)ctx;
	const struct icp201xx_config* cfg = (const struct icp201xx_config*)dev->config;
	return i2c_burst_write_dt(&cfg->i2c, reg, (uint8_t*)wbuffer, wlen);
}
#endif


#define ATMOSPHERICAL_PRESSURE_KPA 101.325
#define TO_KELVIN(temp_C) (273.15 + temp_C)
#define HEIGHT_TO_PRESSURE_COEFF 0.03424 // M*g/R = (0,0289644 * 9,80665 / 8,31432)

#define PRESSURE_TO_HEIGHT_COEFF 29.27127 // R / (M*g) = 8,31432 / (0,0289644 * 9,80665)
#define LOG_ATMOSPHERICAL_PRESSURE 4.61833 // ln(101.325)

float convertToHeight(float pressure_kp, float temperature_C)
{
	return PRESSURE_TO_HEIGHT_COEFF * TO_KELVIN(temperature_C) * (LOG_ATMOSPHERICAL_PRESSURE - log(pressure_kp));
}

/* ICP201xx warm up. 
 * If FIR filter is enabled, it will cause a settling effect on the first 14 pressure values. 
 * Therefore the first 14 pressure output values are discarded.
 **/
static void inv_icp201xx_app_warmup(inv_icp201xx_t* icp_device, icp201xx_op_mode_t op_mode, icp201xx_meas_mode_t meas_mode)
{
  volatile uint8_t fifo_packets = 0;
  uint8_t fifo_packets_to_skip = 14;

  do{
    fifo_packets = 0;
    if ( !inv_icp201xx_get_fifo_count(icp_device,(uint8_t*)&fifo_packets) && ( fifo_packets >= fifo_packets_to_skip ) )
    {
      uint8_t i_status = 0;
      inv_icp201xx_flush_fifo(icp_device);

      inv_icp201xx_get_int_status(icp_device,&i_status);
      if ( i_status )
        inv_icp201xx_clear_int_status(icp_device,i_status);
      break;
    }
    inv_icp201xx_sleep_us(2000);
  } while (1);

}

static int icp201xx_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	int err = 0;
	icp201xx_data* data = (icp201xx_data*)dev->data;
	__ASSERT_NO_MSG(val != NULL);
	if(chan == SENSOR_CHAN_PRESS)
	{
		if (attr == SENSOR_ATTR_CONFIGURATION) {
			if ((val->val1 >= ICP201XX_OP_MODE0)&&(val->val1 <= ICP201XX_OP_MODE4)) {
				err = inv_icp201xx_soft_reset(&(data->icp_device));
				err = inv_icp201xx_config(&(data->icp_device),val->val1,ICP201XX_FIFO_READOUT_MODE_PRES_TEMP);
				inv_icp201xx_app_warmup(&(data->icp_device),ICP201XX_OP_MODE0,ICP201XX_MEAS_MODE_CONTINUOUS);			} else {
				LOG_ERR("Not supported ATTR value");
				return -EINVAL;
			}
			
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}
	};
	return err;
}


static int icp201xx_sample_fetch(const struct device *dev, const enum sensor_channel chan)
{
	icp201xx_data *data = (icp201xx_data*)dev->data;
	uint8_t fifo_packets;
	uint8_t fifo_data[6];
	/** Read measurements count in FIFO **/
	if ( inv_icp201xx_get_fifo_count(&(data->icp_device),&fifo_packets) )
		return -2;

	if (fifo_packets)
	{
		inv_icp201xx_get_fifo_data(&(data->icp_device),1,fifo_data);
#if ICP201XX_BUS_I2C
		/* Perform dummy read to 0x00 register as last transaction after FIFO read for I2C interface */
		do{
			uint8_t dummy_reg = 0;
			inv_io_hal_read_reg(dev, 0, &dummy_reg, 1);
		}while(0);
#endif
		inv_icp201xx_process_raw_data(&(data->icp_device),1,fifo_data,&data->raw_pressure,&data->raw_temperature);

		return 0;
	}
	return -1;
}


static int icp201xx_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	int raw_pressure, raw_temperature;
	float temperature,pressure;
	icp201xx_data* data = (icp201xx_data*)dev->data;

	if (!(chan == SENSOR_CHAN_AMBIENT_TEMP || chan == SENSOR_CHAN_PRESS || SENSOR_CHAN_ALTITUDE)) {
		return -ENOTSUP;
	}


	if ((chan == SENSOR_CHAN_PRESS)||(chan == SENSOR_CHAN_ALTITUDE)) {
		/** P = (POUT/2^17)*40kPa + 70kPa **/
		raw_pressure = data->raw_pressure;
		if (raw_pressure & 0x080000 )
			raw_pressure |= 0xFFF00000;
		pressure = ((float)(raw_pressure) *40 /131072) +70;
	}
	if ((chan == SENSOR_CHAN_AMBIENT_TEMP)||(chan == SENSOR_CHAN_ALTITUDE)) {
		/* T = (TOUT/2^18)*65C + 25C */
		raw_temperature = data->raw_temperature;
		if (raw_temperature & 0x080000 )
			raw_temperature |= 0xFFF00000;
		temperature = ((float)( raw_temperature )*65 /262144 ) + 25;
	}
	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {

		val->val1 = temperature;
		val->val2 = (temperature - val->val1) *1000000;
	} else if (chan == SENSOR_CHAN_PRESS) {
		val->val1 = pressure;
		val->val2 = (pressure - val->val1) *1000000;
	} else if (chan == SENSOR_CHAN_ALTITUDE) {
		float altitude = convertToHeight(pressure,temperature);
		val->val1 = altitude;
		val->val2 = (altitude - val->val1) *1000000;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int icp201xx_init(const struct device *dev)
{
	icp201xx_data* data = (icp201xx_data*)dev->data;
	inv_icp201xx_serif_t icp_serif;
	int rc = 0;
	uint8_t who_am_i;
	uint8_t icp_version;

LOG_INF("ICI");
#if ICP201XX_BUS_I2C
	icp_serif.if_mode = ICP201XX_IF_I2C;
#else
	icp_serif.if_mode = ICP201XX_IF_4_WIRE_SPI;
#endif
	/* Initialize serial interface between MCU and Icp201xx */
	icp_serif.context   = (void*)dev;        /* no need */
	icp_serif.read_reg  = inv_io_hal_read_reg;
	icp_serif.write_reg = inv_io_hal_write_reg;
	icp_serif.max_read  = 2048; /* maximum number of bytes allowed per serial read */
	icp_serif.max_write = 2048; /* maximum number of bytes allowed per serial write */

	rc = inv_icp201xx_init(&(data->icp_device), &icp_serif);
	if (rc != INV_ERROR_SUCCESS) {
		LOG_ERR("Init error");
		return rc;
	}
LOG_INF("LA");

	rc = inv_icp201xx_soft_reset(&(data->icp_device));
LOG_INF("LA");
	if (rc != INV_ERROR_SUCCESS) {
		LOG_ERR("Reset error");
		return rc;
	}
LOG_INF("HERE");

	/* Check WHOAMI */
	rc =  inv_icp201xx_get_devid_version(&(data->icp_device),&who_am_i,&icp_version);
	if(rc != 0) {
		LOG_ERR("Device id error");
		return -2;
	}
	LOG_INF("THERE");

	if (who_am_i != EXPECTED_DEVICE_ID) {
		LOG_ERR("Wrong device id");
		return -3;
	}
LOG_INF("HERE AGAIN");

	/* Boot up OTP config */
	rc = inv_icp201xx_OTP_bootup_cfg(&(data->icp_device));
	if(rc != 0) {
		LOG_ERR("Bootup error");
		return rc;
	}
	rc = inv_icp201xx_soft_reset(&(data->icp_device));
	if(rc != 0) {
		LOG_ERR("Reset error");
		return rc;
	}
	rc = inv_icp201xx_config(&(data->icp_device),ICP201XX_OP_MODE0,ICP201XX_FIFO_READOUT_MODE_PRES_TEMP);
	if(rc != 0) {
		LOG_ERR("Config error");
		return rc;
	}
	inv_icp201xx_app_warmup(&(data->icp_device),ICP201XX_OP_MODE0,ICP201XX_MEAS_MODE_CONTINUOUS);
	// successful init, return 0
	return 0;
}

static const struct sensor_driver_api icp201xx_api_funcs = {
	.sample_fetch = icp201xx_sample_fetch,
	.channel_get = icp201xx_channel_get,
	.attr_set = icp201xx_attr_set,
	.trigger_set = icp201xx_trigger_set
};

#define ICP201XX_DEFINE(inst)                                                                      \
	static icp201xx_data icp201xx_drv_##inst;                                           \
	static const struct icp201xx_config icp201xx_config_##inst = {                         \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.gpio_int = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                                        \
	};                                                                                      \
	DEVICE_DT_INST_DEFINE(inst, icp201xx_init, NULL, &icp201xx_drv_##inst,                     \
			      &icp201xx_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,   \
			      &icp201xx_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(ICP201XX_DEFINE)

