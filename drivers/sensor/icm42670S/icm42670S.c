/*
 * Copyright (c) 2023 TDK Invensense
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

#include "icm42670S.h"

LOG_MODULE_REGISTER(ICM42670S, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "ICM42670S driver enabled without any devices"
#endif



static inline int icm42670S_bus_check(const struct device *dev)
{
	const struct icm42670S_config *cfg = dev->config;

	return cfg->bus_io->check(&cfg->bus);
}

static inline int icm42670S_reg_read(const struct device *dev,
				  uint8_t start, uint8_t *buf, int size)
{
	const struct icm42670S_config *cfg = dev->config;

	return cfg->bus_io->read(&cfg->bus, start, buf, size);
}

static inline int icm42670S_reg_write(const struct device *dev, uint8_t reg,
				   uint8_t val)
{
	const struct icm42670S_config *cfg = dev->config;

	return cfg->bus_io->write(&cfg->bus, reg, val);
}

static int icm42670S_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct icm42670S_data *data = dev->data;
	int size = 6;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;
	(void)pm_device_state_get(dev, &state);
	/* Do not allow sample fetching from suspended state */
	if (state == PM_DEVICE_STATE_SUSPENDED)
		return -EIO;
#endif

	if (data->chip_id == 0x55) {
		size = 8;
	}
	return 0;
}

static int icm42670S_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	//TBD struct icm42670S_data *data = dev->data;

	switch (chan) {

	default:
		return -ENOTSUP;
	}

	return 0;
}

int icm42670S_tap_fetch(const struct device *dev)
{
	/*int result = 0;
	struct icm42605_data *drv_data = dev->data;
	const struct icm42605_config *cfg = dev->config;

	if (drv_data->tap_en &&
	    (drv_data->tap_handler || drv_data->double_tap_handler)) {
		result = inv_spi_read(&cfg->spi, REG_INT_STATUS3, drv_data->fifo_data, 1);
		if (drv_data->fifo_data[0] & BIT_INT_STATUS_TAP_DET) {
			result = inv_spi_read(&cfg->spi, REG_APEX_DATA4,
					      drv_data->fifo_data, 1);
			if (drv_data->fifo_data[0] & APEX_TAP) {
				if (drv_data->tap_trigger->type ==
				    SENSOR_TRIG_TAP) {
					if (drv_data->tap_handler) {
						LOG_DBG("Single Tap detected");
						drv_data->tap_handler(dev
						      , drv_data->tap_trigger);
					}
				} else {
					LOG_ERR("Trigger type is mismatched");
				}
			} else if (drv_data->fifo_data[0] & APEX_DOUBLE_TAP) {
				if (drv_data->double_tap_trigger->type ==
				    SENSOR_TRIG_DOUBLE_TAP) {
					if (drv_data->double_tap_handler) {
						LOG_DBG("Double Tap detected");
						drv_data->double_tap_handler(dev
						     , drv_data->tap_trigger);
					}
				} else {
					LOG_ERR("Trigger type is mismatched");
				}
			} else {
				LOG_DBG("Not supported tap event");
			}
		}
	}*/

	return 0;
}

static const struct sensor_driver_api icm42670S_api_funcs = {
	.sample_fetch = icm42670S_sample_fetch,
	.channel_get = icm42670S_channel_get,
};

static int icm42670S_chip_init(const struct device *dev)
{
	struct icm42670S_data *data = dev->data;
	//TBD const struct icm42670S_config *cfg = dev->config;
	int err;
	uint8_t ReadData;
	
	err = icm42670S_bus_check(dev);
	if (err < 0) {
		LOG_DBG("bus check failed: %d", err);
		return err;
	}
	k_sleep(K_SECONDS(0.3));
	icm42670S_reg_write(dev, 0x79, 0x00);
	icm42670S_reg_write(dev, 0x7C, 0x00);
	// Configure SPI 4 Wire Mode 3
	//icm42670S_reg_write(dev, 0x01, 0x04);
	// Reset
	icm42670S_reg_write(dev, 0x02, 0x10);
	k_sleep(K_SECONDS(1));

	icm42670S_reg_write(dev, 0x79, 0x00);
	icm42670S_reg_write(dev, 0x7C, 0x00);
	// Configure SPI 4 Wire Mode 3
	//icm42670S_reg_write(dev, 0x01, 0x04);
	
	// Clear Interrupt Status RESET_DONE
	icm42670S_reg_read(dev, 0x3A,&ReadData,1);
	if (ReadData != 0x10)
		LOG_DBG("Reset failed: 0x%x", ReadData);
	
	// Test Register Access
	//icm42670S_reg_write(dev, 0x75, 0x55);
	//icm42670S_reg_read(dev, 0x75,&ReadData,1);
	//LOG_DBG("Read Data: 0x%x", ReadData);

	err = icm42670S_reg_read(dev, 0x75, &data->chip_id, 1);
	if (err < 0) {
		LOG_DBG("ID read failed: %d", err);
		return err;
	}

	if (data->chip_id == 0x69) {
		LOG_DBG("ID OK");
	} else {
		LOG_DBG("bad chip id 0x%x", data->chip_id);
		return -ENOTSUP;
	}

	/*err = bme280_reg_write(dev, BME280_REG_RESET, BME280_CMD_SOFT_RESET);
	if (err < 0) {
		LOG_DBG("Soft-reset failed: %d", err);
	}

	err = bme280_wait_until_ready(dev);
	if (err < 0) {
		return err;
	}

	err = bme280_read_compensation(dev);
	if (err < 0) {
		return err;
	}

	if (data->chip_id == BME280_CHIP_ID) {
		err = bme280_reg_write(dev, BME280_REG_CTRL_HUM,
				       BME280_HUMIDITY_OVER);
		if (err < 0) {
			LOG_DBG("CTRL_HUM write failed: %d", err);
			return err;
		}
	}

	err = bme280_reg_write(dev, BME280_REG_CTRL_MEAS,
			       BME280_CTRL_MEAS_VAL);
	if (err < 0) {
		LOG_DBG("CTRL_MEAS write failed: %d", err);
		return err;
	}

	err = bme280_reg_write(dev, BME280_REG_CONFIG,
			       BME280_CONFIG_VAL);
	if (err < 0) {
		LOG_DBG("CONFIG write failed: %d", err);
		return err;
	}
	*/
	k_sleep(K_MSEC(1));

	LOG_DBG("\"%s\" OK", dev->name);
	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int icm42670S_pm_action(const struct device *dev,
			    enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* Re-initialize the chip */
		ret = icm42670S_chip_init(dev);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		/* Put the chip into sleep mode */
		ret = icm42670S_reg_write(dev,
			0x00,//BME280_REG_CTRL_MEAS,
			0x00,//BME280_CTRL_MEAS_OFF_VAL);

		if (ret < 0) {
			LOG_DBG("CTRL_MEAS write failed: %d", ret);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

/* Initializes a struct icm42670S_config for an instance on a SPI bus. */
#define ICM42670S_CONFIG_SPI(inst)				\
	{						\
		.bus.spi = SPI_DT_SPEC_INST_GET(	\
			inst, ICM42670S_SPI_OPERATION, 0),	\
		.bus_io = &icm42670S_bus_io_spi,		\
		.gpio_int = GPIO_DT_SPEC_INST_GET(inst, int_gpios),    \
		.accel_hz = DT_INST_PROP(inst, accel_hz),		\
		.gyro_hz = DT_INST_PROP(inst, gyro_hz),		\
		.accel_fs = DT_INST_ENUM_IDX(inst, accel_fs),		\
		.gyro_fs = DT_INST_ENUM_IDX(inst, gyro_fs),		\
	}

/* Initializes a struct icm42670S_config for an instance on an I2C bus. */
#define ICM42670S_CONFIG_I2C(inst)			       \
	{					       \
		.bus.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.bus_io = &icm42670S_bus_io_i2c,	       \
		.gpio_int = GPIO_DT_SPEC_INST_GET(inst, int_gpios),    \
		.accel_hz = DT_INST_PROP(inst, accel_hz),		\
		.gyro_hz = DT_INST_PROP(inst, gyro_hz),		\
		.accel_fs = DT_INST_ENUM_IDX(inst, accel_fs),		\
		.gyro_fs = DT_INST_ENUM_IDX(inst, gyro_fs),		\
	}

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define ICM42670S_DEFINE(inst)						\
	static struct icm42670S_data icm42670S_data_##inst;			\
	static const struct icm42670S_config icm42670S_config_##inst =	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			    (ICM42670S_CONFIG_SPI(inst)),			\
			    (ICM42670S_CONFIG_I2C(inst)));			\
									\
	PM_DEVICE_DT_INST_DEFINE(inst, icm42670S_pm_action);		\
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			 icm42670S_chip_init,				\
			 PM_DEVICE_DT_INST_GET(inst),			\
			 &icm42670S_data_##inst,				\
			 &icm42670S_config_##inst,				\
			 POST_KERNEL,					\
			 CONFIG_SENSOR_INIT_PRIORITY,			\
			 &icm42670S_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(ICM42670S_DEFINE)
