/*
 * Copyright (c) 2025 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/logging/log.h>

#include "ictx53xx.h"

LOG_MODULE_REGISTER(ICTX53XX, CONFIG_SENSOR_LOG_LEVEL);

void inv_ictx53xx_sleep_us(unsigned int us)
{
	k_usleep(us);
}

static int inv_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
	const struct device *dev = context;
	struct ictx53xx_data *data = dev->data;
	
	return ictx53xx_reg_read_rtio(&data->bus, reg | REG_READ_BIT, rbuffer, rlen);
}

static int inv_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
	const struct device *dev = context;
	struct ictx53xx_data *data = dev->data;
	
	return ictx53xx_reg_write_rtio(&data->bus, reg, wbuffer, wlen);
}

static int ictx53xx_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	return 0;
}

static int ictx53xx_attr_get(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, struct sensor_value *val)
{
	return 0;
}

static int ictx53xx_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct ictx53xx_data *data = (struct ictx53xx_data *)dev->data;
	int drdy_status;
	int16_t mag_data[3];
	int ret = 0;

	switch (chan) {
	case SENSOR_CHAN_ALL:
	case SENSOR_CHAN_MAGN_XYZ:
	case SENSOR_CHAN_DIE_TEMP:
		ret |= inv_ict_get_data_ready_status(&data->driver, &drdy_status);
		if (drdy_status == 1)
			ret |= inv_ict_poll_data(&data->driver, mag_data, &data->temp);
		else 
			ret = 1;
		data->x = mag_data[0];
		data->y = mag_data[1];
		data->z = mag_data[2];
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}

static void ictx53xx_convert_mag(struct ictx53xx_data *data, struct sensor_value *val, int16_t sample)
{
	uint32_t sensitivity;
	int64_t conv_val;
	
	inv_ict_get_sensitivity_nt_per_lsb(&data->driver, &sensitivity);
	/* Convert nT/LSB to µG/LSB */
	conv_val = sample * (sensitivity * 10);

	val->val1 = conv_val / 1000000;
	val->val2 = conv_val - (val->val1 * 1000000);
}

static void ictx53xx_convert_temp(struct sensor_value *val, int16_t sample)
{
	/* Convert temp to C (degC = lsb * 6.25 / 1000 + 25) */
	int64_t conv_val = (int64_t)sample * 625 / 100000 + 25;

	val->val1 = conv_val / 1000000;
	val->val2 = conv_val - (val->val1 * 1000000);
}

static int ictx53xx_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct ictx53xx_data *data = (struct ictx53xx_data *)dev->data;

	switch (chan) {
	case SENSOR_CHAN_MAGN_XYZ:
		ictx53xx_convert_mag(data, val, data->x);
		ictx53xx_convert_mag(data, val + 1, data->y);
		ictx53xx_convert_mag(data, val + 2, data->z);
		break;
	case SENSOR_CHAN_MAGN_X:
		ictx53xx_convert_mag(data, val, data->x);
		break;
	case SENSOR_CHAN_MAGN_Y:
		ictx53xx_convert_mag(data, val, data->y);
		break;
	case SENSOR_CHAN_MAGN_Z:
		ictx53xx_convert_mag(data, val, data->z);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		ictx53xx_convert_temp(val, data->temp);
	default:
		return ENOTSUP;
	}
	return 0;
}

static int ictx53xx_init(const struct device *dev)
{
	struct ictx53xx_data *data = (struct ictx53xx_data *)dev->data;
	inv_ict_serif_t ict_serif;
	inv_ict_id_t id;
	int rc = 0;

	ict_serif.context = (struct device *)dev;
	ict_serif.read_reg = inv_io_hal_read_reg;
	ict_serif.write_reg = inv_io_hal_write_reg;
	ict_serif.sleep_us = inv_ictx53xx_sleep_us;

	/* Init ICT */
	rc |= inv_ict_init(&data->driver, &ict_serif);
	if (rc != 0) {
		LOG_ERR("Failed to initialize ict_dev.");
		return rc;
	}

	/* Check ID */
	rc = inv_ict_get_id(&data->driver, &id);
	if (rc != 0) {
		LOG_ERR("Failed to read ICT ID.");
		return rc;
	}

	switch (id) {
	case ICT1531X:
		LOG_INF("> ICT1531X detected");
		break;
	case ICT25324:
		LOG_INF("> ICT25324 detected");
		break;
	case ICT25349:
		LOG_INF("> ICT25349 detected");
		break;
	default:
		LOG_ERR("Unknown ID for mag device.");
		rc = -EINVAL;
		break;
	}

	LOG_INF("Magnetometer successfully initialized");

	/* successful init, return 0 */
	return rc;
}

static DEVICE_API(sensor, ictx53xx_api_funcs) = {
	.sample_fetch = ictx53xx_sample_fetch,
	.channel_get = ictx53xx_channel_get,
	.attr_set = ictx53xx_attr_set,
	.attr_get = ictx53xx_attr_get,
#if defined(CONFIG_SENSOR_ASYNC_API)
	// .get_decoder = ictx53xx_get_decoder,
	// .submit = ictx53xx_submit,
#endif /* CONFIG_SENSOR_ASYNC_API */
};

/*
 * Main instantiation macro
 */
#define ICTX53XX_DEFINE(inst)          \
	I2C_DT_IODEV_DEFINE(ictx53xx_bus_##inst, DT_DRV_INST(inst));   \
	RTIO_DEFINE(ictx53xx_rtio_ctx_##inst, 32, 32);          \
																	\
	static const struct ictx53xx_config	ictx53xx_config_##inst = {  \
		.op_mode = INV_ICT_MODE_CTRL_REG_MODE_SINGLE_SHOT,           \
	};                                                            \
	static struct ictx53xx_data ictx53xx_drv_##inst = {          \
		.bus = {                                                  \
				.rtio_ctx = &ictx53xx_rtio_ctx_##inst,          \
				.iodev = &ictx53xx_bus_##inst,  },              \
	};                                                            \
																	\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ictx53xx_init, NULL, &ictx53xx_drv_##inst, \
		&ictx53xx_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,   \
		&ictx53xx_api_funcs);

#define DT_DRV_COMPAT invensense_ictx53xx
DT_INST_FOREACH_STATUS_OKAY(ICTX53XX_DEFINE)
