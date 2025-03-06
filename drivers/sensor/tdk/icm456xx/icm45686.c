/*
 * Copyright (c) 2025 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "icm45686.h"

#include <zephyr/drivers/sensor/icm456xx.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ICM45686, CONFIG_SENSOR_LOG_LEVEL);

static uint32_t convert_gyr_fs_to_enum(uint32_t val)
{
	uint32_t fs_enum = 0;

	if (val < 500 && val >= 250) {
		fs_enum = GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS;
	} else if (val < 1000 && val >= 500) {
		fs_enum = GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS;
	} else if (val < 2000 && val >= 1000) {
		fs_enum = GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS;
	} else if (val == 2000) {
		fs_enum = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;
	}
	return fs_enum;
}

static int icm45686_set_gyro_odr(struct icm456xx_data *drv_data, const struct sensor_value *val)
{
	return 0;
}

static int icm45686_set_gyro_fs(struct icm456xx_data *drv_data, const struct sensor_value *val)
{
	int32_t val_dps = sensor_rad_to_degrees(val);

	return 0;
}

int icm45686_gyro_config(struct icm456xx_data *drv_data, enum sensor_attribute attr,
			 const struct sensor_value *val)
{
	return 0;
}

void icm45686_convert_gyro(struct sensor_value *val, int16_t raw_val, uint16_t fs)
{
	int64_t conv_val;

	/* 16 bit gyroscope. 2^15 bits represent the range in degrees/s */
	/* see datasheet section 3.1 for details */
	conv_val = ((int64_t)raw_val * fs * SENSOR_PI) / (INT16_MAX * 180U);

	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

int icm45686_sample_fetch_gyro(const struct device *dev)
{
	LOG_ERR("fetch gyro");
	return 0;
}

uint16_t convert_enum_to_gyr_fs(uint8_t fs_enum)
{
	uint16_t gyr_fs = 0;

	if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS) {
		gyr_fs = 250;
	} else if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS) {
		gyr_fs = 500;
	} else if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS) {
		gyr_fs = 1000;
	} else if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS) {
		gyr_fs = 2000;
	}
	return gyr_fs;
}
