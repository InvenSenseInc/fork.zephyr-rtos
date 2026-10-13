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
	struct ictx53xx_data *data = dev->data;

	__ASSERT_NO_MSG(val != NULL);
	
	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
		if (attr == SENSOR_ATTR_CONFIGURATION) {
			if ((val->val1 == INV_ICT_MODE_CTRL_REG_MODE_SLEEP) || (val->val1 == INV_ICT_MODE_CTRL_REG_MODE_PULSED)
						|| (val->val1 == INV_ICT_MODE_CTRL_REG_MODE_SINGLE_SHOT)) {
				data->mode = val->val1;
				inv_ict_set_mode(&data->driver, data->mode);
			} else if (data->mode == INV_ICT_MODE_CTRL_REG_MODE_MRM) {
				/* Ensure Sleep mode before setting MRM */
				inv_ict_set_mode(&data->driver, INV_ICT_MODE_CTRL_REG_MODE_SLEEP);
				/* MRM enables magnetic reset of TMR sensor element with a pulsed current. */
				inv_ict_set_mrm(&data->driver);
				/* Sleep mode is forced at the end of MRN operation */
				data->mode = INV_ICT_MODE_CTRL_REG_MODE_SLEEP;
			} else {
				LOG_ERR("Not supported ATTR value");
				return -EINVAL;
			}
		} else if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			data->odr = ictx53xx_hz_to_reg(val->val1);
			inv_ict_set_odr(&data->driver, data->odr);
		} else {
			LOG_ERR("Invalid attribute %d", attr);
			return -EINVAL;
		}
		break;
	default:
		LOG_ERR("Invalid channel %d", chan);
		(void)data;
		return -EINVAL;
	}
	return 0;
}

static int ictx53xx_attr_get(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, struct sensor_value *val)
{
	struct ictx53xx_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
		if (attr == SENSOR_ATTR_CONFIGURATION) {
			val->val1 = data->mode;
			val->val2 = 0;
		} else if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			if (data->mode == INV_ICT_MODE_CTRL_REG_MODE_PULSED) {
				ictx53xx_reg_to_hz(data->odr, val);
			} else {
				val->val1 = 0;
				val->val2 = 0;
			}
		} else {
			LOG_ERR("Invalid attribute %d", attr);
			return -EINVAL;
		}
		break;
	default:
		LOG_ERR("Invalid channel %d", chan);
		(void)data;
		return -EINVAL;
	}
	return 0;
}

static int ictx53xx_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct ictx53xx_data *data = (struct ictx53xx_data *)dev->data;
	int drdy_status;
	int16_t mag_data[3];
	int16_t temp_data;
	int8_t trials = 6;
	int ret = 0;

	switch (chan) {
	case SENSOR_CHAN_ALL:
	case SENSOR_CHAN_MAGN_XYZ:
	case SENSOR_CHAN_AMBIENT_TEMP:
		if (data->mode == INV_ICT_MODE_CTRL_REG_MODE_PULSED) {
			ret |= inv_ict_get_data_ready_status(&data->driver, &drdy_status);
		} else {
			ret |= inv_ict_set_mode(&data->driver, INV_ICT_MODE_CTRL_REG_MODE_SINGLE_SHOT);
			
			/* Initial sleep waiting the sensor proceeds with the measure = 3050us */
			k_sleep(K_USEC(ICTX53XX_MEASUREMENT_TIME_US));
			do {
				k_sleep(K_USEC(ICTX53XX_MEASUREMENT_EXTRA_TIME_US));
				ret |= inv_ict_get_data_ready_status(&data->driver, &drdy_status);
			} while ((drdy_status != 1) && (trials-- > 0));
		}
		break;
	default:
		return -ENOTSUP;
	}

	if (drdy_status) {
		ret |= inv_ict_poll_data(&data->driver, mag_data, &temp_data);
		data->edata.mag.x = mag_data[0];
		data->edata.mag.y = mag_data[1];
		data->edata.mag.z = mag_data[2];
		data->edata.temp = temp_data;
	}
	return ret;
}

int32_t ictx53xx_convert_mag(int16_t sample)
{
	/* ICTx53xx magnetometer sensitivity in nT converted in µG */
	return ((int32_t)sample * ICTX53XX_SENSITIVITY * 10);
}

int32_t ictx53xx_convert_temp(int16_t sample)
{
	/* Convert to microcelsius */
	return ((int32_t)sample * 6250 + 25000000);
}

static int ictx53xx_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct ictx53xx_encoded_data *edata = (struct ictx53xx_encoded_data *)dev->data;

	switch (chan) {
	case SENSOR_CHAN_MAGN_XYZ:
		/* Convert raw magnetic data to gauss */
		sensor_value_from_micro(val, ictx53xx_convert_mag(edata->mag.x));
		sensor_value_from_micro(val + 1, ictx53xx_convert_mag(edata->mag.y));
		sensor_value_from_micro(val + 2, ictx53xx_convert_mag(edata->mag.z));
		break;
	case SENSOR_CHAN_MAGN_X:
		sensor_value_from_micro(val, ictx53xx_convert_mag(edata->mag.x));
		break;
	case SENSOR_CHAN_MAGN_Y:
		sensor_value_from_micro(val, ictx53xx_convert_mag(edata->mag.y));
		break;
	case SENSOR_CHAN_MAGN_Z:
		sensor_value_from_micro(val, ictx53xx_convert_mag(edata->mag.z));
		break;
	case SENSOR_CHAN_AMBIENT_TEMP:
		sensor_value_from_micro(val, ictx53xx_convert_temp(edata->temp));
		return 0;
	default:
		return ENOTSUP;
	}
	return 0;
}

static int ictx53xx_init(const struct device *dev)
{
	struct ictx53xx_data *data = (struct ictx53xx_data *)dev->data;
	const struct ictx53xx_config *cfg = dev->config;
	inv_ict_serif_t ict_serif;
	inv_ict_id_t id;
	int rc = 0;

	ict_serif.context = (struct device *)dev;
	ict_serif.read_reg = inv_io_hal_read_reg;
	ict_serif.write_reg = inv_io_hal_write_reg;
	ict_serif.sleep_us = inv_ictx53xx_sleep_us;
	ict_serif.max_read  = 21;
	ict_serif.max_write = 6;

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

	/* Apply dt-binding settings */
	data->odr = ictx53xx_hz_to_reg(cfg->odr_hz);
	inv_ict_set_odr(&data->driver, data->odr);
 
	data->mode = cfg->mode;
	inv_ict_set_mode(&data->driver, data->mode);
	if (data->mode == INV_ICT_MODE_CTRL_REG_MODE_MRM) {
		inv_ict_set_mrm(&data->driver);
		/* Sleep mode is forced at the end of MRN operation */
		data->mode = INV_ICT_MODE_CTRL_REG_MODE_SLEEP;
	}

#ifdef CONFIG_SENSOR_ASYNC_API
	/* init work for fetching after measurement has completed */
	k_work_init_delayable(&data->work_ctx.async_fetch_work, ictx53xx_async_fetch);
#endif
	
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
	.get_decoder = ictx53xx_get_decoder,
	.submit = ictx53xx_submit,
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
			.mode = DT_INST_ENUM_IDX(inst, mode),                 \
			.odr_hz = DT_INST_ENUM_IDX(inst, odr_hz),                 \
	};                                                            \
	static struct ictx53xx_data ictx53xx_drv_##inst = {         	 \
		.mode = INV_ICT_MODE_CTRL_REG_MODE_SLEEP,					\
		.odr = 0,					\
		.bus = {                                                  \
				.rtio_ctx = &ictx53xx_rtio_ctx_##inst,          \
				.iodev = &ictx53xx_bus_##inst,  },              \
	};                                                            \
																	\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ictx53xx_init, NULL, &ictx53xx_drv_##inst, \
		&ictx53xx_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,   \
		&ictx53xx_api_funcs);

#define DT_DRV_COMPAT invensense_ictx53xx
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY(ICTX53XX_DEFINE)
#endif
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT invensense_ict25349
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY(ICTX53XX_DEFINE)
#endif
#undef DT_DRV_COMPAT
