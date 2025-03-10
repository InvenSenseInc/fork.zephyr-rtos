/*
 * Copyright (c) 2025 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM45686_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM45686_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include "icm456xx.h"

int icm45686_gyro_config(struct icm456xx_data *drv_data, enum sensor_attribute attr,
			 const struct sensor_value *val);
void icm45686_convert_gyro(struct sensor_value *val, int16_t raw_val, uint16_t fs);
int icm45686_sample_fetch_gyro(const struct device *dev);
uint16_t convert_enum_to_gyr_fs(uint8_t fs_enum);

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM45686_H_ */
