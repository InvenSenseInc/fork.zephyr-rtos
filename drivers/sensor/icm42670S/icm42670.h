/*
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM42670_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM42670_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>

#define DT_DRV_COMPAT tdk_invensense_icm42670S

#define ICM42670S_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define ICM42670S_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

union icm42670S_config {
#if BME280_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if BME280_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
	struct gpio_dt_spec gpio_int;
};

struct icm42670_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t accel_sensitivity_shift;
	uint16_t accel_hz;
	uint16_t accel_fs;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint16_t gyro_sensitivity_x10;
	uint16_t gyro_hz;
	uint16_t gyro_fs;
	int16_t temp;
#ifdef CONFIG_ICM42670_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t data_ready_handler;
	const struct sensor_trigger *data_ready_trigger;
	struct k_mutex mutex;
#endif
#ifdef CONFIG_ICM42670_TRIGGER_OWN_THREAD
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ICM42670_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#endif
#ifdef CONFIG_ICM42670_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif
};


#endif /* ZEPHYR_DRIVERS_SENSOR_ICM42670_H_ */
