/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICT1531X_H_
#define ZEPHYR_DRIVERS_SENSOR_ICT1531X_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include "Ict1531x.h"
#include "Ict1531xSerif.h"

#define DT_DRV_COMPAT invensense_ict1531x

#define ICT1531X_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

struct ict1531x_data {
	int16_t raw_mag[3];
	int16_t raw_temperature;
	struct inv_ict1531x ict_device;
};

struct ict1531x_config {
	struct i2c_dt_spec i2c;
};

#endif
