/*
 * Copyright (c) 2023 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM42670S_ICM42670S_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM42670S_ICM42670S_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#include "imu/inv_imu_driver.h"
#ifdef CONFIG_ICM42670S_APEX
#include "imu/inv_imu_apex.h"
#endif

#define DT_DRV_COMPAT invensense_icm42670s

#define ICM42670S_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define ICM42670S_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

union icm42670S_bus {
#if ICM42670S_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if ICM42670S_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
};

typedef void (*tap_fetch_t)(const struct device *dev);
int icm42670S_tap_fetch(const struct device *dev);

typedef int (*icm42670S_bus_check_fn)(const union icm42670S_bus *bus);
typedef int (*icm42670S_reg_read_fn)(const union icm42670S_bus *bus,
				  uint8_t reg, uint8_t *buf, uint32_t size);
typedef int (*icm42670S_reg_write_fn)(const union icm42670S_bus *bus,
				   uint8_t reg, uint8_t *buf, uint32_t size);

struct icm42670S_bus_io {
	icm42670S_bus_check_fn check;
	icm42670S_reg_read_fn read;
	icm42670S_reg_write_fn write;
};

#if ICM42670S_BUS_SPI
#define ICM42670S_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB |	\
			      SPI_MODE_CPOL | SPI_MODE_CPHA)
extern const struct icm42670S_bus_io icm42670S_bus_io_spi;
#endif

#if ICM42670S_BUS_I2C
extern const struct icm42670S_bus_io icm42670S_bus_io_i2c;
#endif

struct icm42670S_data {
	struct inv_imu_serif serif;
	struct inv_imu_device driver;
	uint8_t chip_id;
	int32_t accel[3];
	int32_t gyro[3];
	int32_t temperature;

	uint8_t accel_fs;
	uint16_t gyro_fs;
	
#ifdef CONFIG_ICM42670S_APEX_PEDOMETER
	uint8_t dmp_odr_hz;
	uint64_t pedometer_cnt;
	uint8_t pedometer_activity;
	uint8_t pedometer_cadence;
#endif
#ifdef CONFIG_ICM42670S_APEX_WOM
	uint8_t wom_x;
	uint8_t wom_y;
	uint8_t wom_z;
#endif

	const struct device *dev;
	struct gpio_callback gpio_cb;

	const struct sensor_trigger *data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ICM42670S_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
};

struct icm42670S_config {
	union icm42670S_bus bus;
	const struct icm42670S_bus_io *bus_io;
	struct gpio_dt_spec gpio_int;	
};

int icm42670S_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);

int icm42670S_init_interrupt(const struct device *dev);

#ifdef CONFIG_ICM42670S_APEX_PEDOMETER
int icm42670S_apex_enable(inv_imu_device_t *s);
int icm42670S_apex_enable_pedometer(const struct device *dev, inv_imu_device_t *s);
int icm42670S_apex_pedometer_fetch_from_dmp(const struct device *dev);
void icm42670S_apex_pedometer_cadence_convert(struct sensor_value *val, uint8_t raw_val, uint8_t dmp_odr_hz);
#endif

#ifdef CONFIG_ICM42670S_APEX_TILT
int icm42670S_apex_enable(inv_imu_device_t *s);
int icm42670S_apex_enable_tilt(inv_imu_device_t *s);
int icm42670S_apex_tilt_fetch_from_dmp(const struct device *dev);
#endif

#ifdef CONFIG_ICM42670S_APEX_SMD
int icm42670S_apex_enable(inv_imu_device_t *s);
int icm42670S_apex_enable_smd(inv_imu_device_t *s);
int icm42670S_apex_smd_fetch_from_dmp(const struct device *dev);
#endif

#ifdef CONFIG_ICM42670S_APEX_WOM
int icm42670S_apex_enable_wom(inv_imu_device_t *s);
int icm42670S_apex_wom_fetch_from_dmp(const struct device *dev);
#endif

#ifdef CONFIG_ICM42670S_AML
int icm42670S_aml_init(inv_imu_device_t *s, int8_t delta_gain_x, int8_t delta_gain_y);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM42670S_ICM42670S_H_ */
