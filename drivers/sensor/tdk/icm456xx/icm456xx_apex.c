/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "icm456xx.h"
#include "imu/inv_imu_edmp.h"

int icm456xx_apex_enable(inv_imu_device_t *s)
{
	return 0;
}

int icm456xx_apex_fetch_from_dmp(const struct device *dev)
{
	return 0;
}

void icm456xx_apex_pedometer_cadence_convert(struct sensor_value *val, uint8_t raw_val,
					     uint8_t dmp_odr_hz)
{
	int64_t conv_val;

	/* Converting u6.2 */
	conv_val = (int64_t)(dmp_odr_hz << 2) * 1000000 / (raw_val + (raw_val & 0x03));
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

int icm456xx_apex_enable_pedometer(const struct device *dev, inv_imu_device_t *s)
{
	return 0;
}

int icm456xx_apex_enable_tilt(inv_imu_device_t *s)
{
	return 0;
}

int icm456xx_apex_enable_smd(inv_imu_device_t *s)
{
	return 0;
}

int icm456xx_apex_enable_wom(inv_imu_device_t *s)
{
	return 0;
}
