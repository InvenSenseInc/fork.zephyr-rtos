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
	const struct icp201xx_config* cfg = (const struct icp101xx_config*)dev->config;
	return i2c_read_dt(&cfg->i2c, (uint8_t*)rbuffer, rlen);
}

static int inv_io_hal_write_reg(void* ctx, uint8_t reg, 
                         const uint8_t *wbuffer, uint32_t wlen)
{
	struct device* dev = (struct device*)ctx;
	const struct icp201xx_config* cfg = (const struct icp101xx_config*)dev->config;
	return i2c_write_dt(&cfg->i2c, (uint8_t*)wbuffer, wlen);
}
#endif

static uint8_t get_timeout_ms(enum icp201xx_meas mode)
{
	switch (mode)
	{
		case ICP201XX_OP_MODE0:
			return 2;
		break;
		case ICP201XX_OP_MODE1:
			return 7;
		break;
		case ICP201XX_OP_MODE2:
			return 24;
		break;
		case ICP201XX_OP_MODE3:
			return 95;
		break;
		default:
		case ICP201XX_OP_MODE4:
			return 95;
		break;
	}
}

#define ATMOSPHERICAL_PRESSURE_KPA 101.325
#define TO_KELVIN(temp_C) (273.15 + temp_C)
#define HEIGHT_TO_PRESSURE_COEFF 0.03424 // M*g/R = (0,0289644 * 9,80665 / 8,31432)

#define PRESSURE_TO_HEIGHT_COEFF 29.27127 // R / (M*g) = 8,31432 / (0,0289644 * 9,80665)
#define LOG_ATMOSPHERICAL_PRESSURE 4.61833 // ln(101.325)

float convertToHeight(float pressure_kp, float temperature_C)
{
	return PRESSURE_TO_HEIGHT_COEFF * TO_KELVIN(temperature_C) * (LOG_ATMOSPHERICAL_PRESSURE - log(pressure_kp));
}

static int icp201xx_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	int err = 0;
	icp101xx_data* data = (icp101xx_data*)dev->data;
	__ASSERT_NO_MSG(val != NULL);
	if(chan == SENSOR_CHAN_PRESS)
	{
		if (attr == SENSOR_ATTR_CONFIGURATION) {
			if ((val->val1 >= ICP101XX_MEAS_LOW_POWER_T_FIRST)&&(val->val1 <= ICP101XX_MEAS_ULTRA_LOW_NOISE_P_FIRST)) {
				data->icp_device.measurement_mode = val->val1;
			} else {
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
	icp101xx_data *data = (icp101xx_data*)dev->data;
	int rc = 0;
	uint64_t timeout;

	if (!(chan == SENSOR_CHAN_AMBIENT_TEMP || chan == SENSOR_CHAN_PRESS || SENSOR_CHAN_ALTITUDE ||
		chan == SENSOR_CHAN_ALL)) {
		return -ENOTSUP;
	}
	rc = inv_icp101xx_enable_sensor(&data->icp_device, 1);
	// Compute timeout for the measure
	timeout = k_uptime_get() + get_timeout_ms(data->icp_device.measurement_mode);
	// Initial sleep waiting the sensor proceeds with the measure
	k_sleep(K_MSEC(get_conversion_ms(data->icp_device.measurement_mode)));
	do {
		k_sleep(K_USEC(200));
		rc = inv_icp101xx_get_data(&data->icp_device, &(data->raw_pressure), &(data->raw_temperature), &(data->pressure), &(data->temperature));
	} while ((rc != 0) && (k_uptime_get() <= timeout));
	// Zephyr expects kPa while ICP101xx returns Pa
	data->pressure = data->pressure/1000;
	return rc;
}

static int icp201xx_sample_fetch(const struct device *dev, const enum sensor_channel chan)
	icp201xx_data *data = (icp201xx_data*)dev->data;
	uint8_t fifo_packets;
	uint8_t fifo_data[6];
	int32_t data_press,data_temp;
	/** Read measurements count in FIFO **/
	if ( inv_icp201xx_get_fifo_count(&icp_device,&fifo_packets) )
		return -2;

	if (fifo_packets)
	{
		inv_icp201xx_get_fifo_data(&icp_device,1,fifo_data);
#if ICP201XX_BUS_I2C
		/* Perform dummy read to 0x00 register as last transaction after FIFO read for I2C interface */
		do{
			uint8_t dummy_reg = 0;
			i2c_read(&icp_device, 0, &dummy_reg, 1);
		}while(0);
#endif
		inv_icp201xx_process_raw_data(&icp_device,1,fifo_data,&data_press,&data_temp);

		/** P = (POUT/2^17)*40kPa + 70kPa **/
		if (data_press & 0x080000 )
			data_press |= 0xFFF00000;
		data->pressure = ((float)(data_press) *40 /131072) +70;
//		data->pressure) = round(data->pressure * 1000.0)/1000.0;

		/* T = (TOUT/2^18)*65C + 25C */
		if (data_temp & 0x080000 )
			data_temp |= 0xFFF00000;
		data->temperature = ((float)( data_temp )*65 /262144 ) + 25;
		data->temperature = round(data->temperature*10.0)/10.0;

		return 0;
	}
	return -1;
}


static int icp201xx_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	icp201xx_data* data = (icp201xx_data*)dev->data;

	if (!(chan == SENSOR_CHAN_AMBIENT_TEMP || chan == SENSOR_CHAN_PRESS || SENSOR_CHAN_ALTITUDE)) {
		return -ENOTSUP;
	}
	
	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		val->val1 = data->temperature;
		val->val2 = (data->temperature - val->val1) *1000000;
	} else if (chan == SENSOR_CHAN_PRESS) {
		val->val1 = data->pressure;
		val->val2 = (data->pressure - val->val1) *1000000;
	} else if (chan == SENSOR_CHAN_ALTITUDE) {
		float altitude = convertToHeight(data->pressure,data->temperature);
		val->val1 = altitude;
		val->val2 = (altitude - val->val1) *1000000;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int icp201xx_init(const struct device *dev)
{
	struct inv_icp101xx_serif icp101xx_serif;
	int rc = 0;
	icp101xx_data* data = (icp101xx_data*)dev->data;
	const struct icp101xx_config* cfg = (const struct icp101xx_config*)dev->config;
	
	icp101xx_serif.context   = (void*)dev;
	icp101xx_serif.read_reg  = inv_io_hal_read_reg;
	icp101xx_serif.write_reg = inv_io_hal_write_reg;
	icp101xx_serif.max_read  = 2048; /* maximum number of bytes allowed per serial read */
	icp101xx_serif.max_write = 2048; /* maximum number of bytes allowed per serial write */
	icp101xx_serif.is_spi    = 0;
	/*
	 * Reset pressure sensor driver states
	 */
	inv_icp101xx_reset_states(&(data->icp_device), &icp101xx_serif);

	rc = inv_icp101xx_soft_reset(&data->icp_device);
	if (rc != 0) {
		LOG_ERR("Soft reset error %d", rc);
		return rc;
	}
	inv_icp101xx_init(&data->icp_device);
	if (rc != 0) {
		LOG_ERR("Init error %d", rc);
		return rc;
	}
	data->icp_device.measurement_mode = cfg->mode;

	// successful init, return 0
	return 0;

}

static const struct sensor_driver_api icp201xx_api_funcs = {
	.sample_fetch = icp201xx_sample_fetch,
	.channel_get = icp201xx_channel_get,
	.attr_set = icp102xx_attr_set,
	.trigger_set = icp201xx_trigger_set
};

#define ICP101XX_DEFINE(inst)                                                                      \
	static icp101xx_data icp101xx_drv_##inst;                                           \
	static const struct icp101xx_config icp101xx_config_##inst = {                         \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.mode = DT_INST_ENUM_IDX(inst, mode),                                             \
	};                                                                                      \
	DEVICE_DT_INST_DEFINE(inst, icp101xx_init, NULL, &icp101xx_drv_##inst,                     \
			      &icp101xx_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,   \
			      &icp101xx_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(ICP101XX_DEFINE)

