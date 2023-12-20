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

struct icm42670S_data {
	int32_t accel[3];
	int32_t gyro[3];
	uint32_t comp_humidity;
	uint8_t chip_id;
};

struct icm42670S_config {
	union icm42670S_bus bus;
	const struct icm42670S_bus_io *bus_io;
};

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
	uint8_t buf[8];
	int32_t adc_press, adc_temp, adc_humidity;
	int size = 6;
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;
	(void)pm_device_state_get(dev, &state);
	/* Do not allow sample fetching from suspended state */
	if (state == PM_DEVICE_STATE_SUSPENDED)
		return -EIO;
#endif

#ifdef CONFIG_BME280_MODE_FORCED
	ret = icm42670S_reg_write(dev, BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS_VAL);
	if (ret < 0) {
		return ret;
	}
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
	struct icm42670S_data *data = dev->data;

	switch (chan) {

	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api icm42670S_api_funcs = {
	.sample_fetch = icm42670S_sample_fetch,
	.channel_get = icm42670S_channel_get,
};

static int icm42670S_chip_init(const struct device *dev)
{
	struct icm42670S_data *data = dev->data;
	int err;

	err = icm42670S_bus_check(dev);
	if (err < 0) {
		LOG_DBG("bus check failed: %d", err);
		return err;
	}

	err = icm42670S_reg_read(dev, 0x55, &data->chip_id, 1);
	if (err < 0) {
		LOG_DBG("ID read failed: %d", err);
		return err;
	}

	if (data->chip_id == 0x55) {
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

/* Initializes a struct icm42670S_config for an instance on a SPI bus. */
#define ICM42670S_CONFIG_SPI(inst)				\
	{						\
		.bus.spi = SPI_DT_SPEC_INST_GET(	\
			inst, ICM42670S_SPI_OPERATION, 0),	\
		.bus_io = &icm42670S_bus_io_spi,		\
	}

/* Initializes a struct icm42670S_config for an instance on an I2C bus. */
#define ICM42670S_CONFIG_I2C(inst)			       \
	{					       \
		.bus.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.bus_io = &icm42670S_bus_io_i2c,	       \
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
