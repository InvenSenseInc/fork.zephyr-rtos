/*
 * Copyright (c) 2024 TDK Invensense
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for ICM456XX accessed via I2C.
 */

#include "icm456xx.h"

static int icm456xx_bus_check_i2c(const union icm456xx_bus *bus)
{
	return device_is_ready(bus->i2c.bus) ? 0 : -ENODEV;
}

static int icm456xx_reg_read_i2c(const union icm456xx_bus *bus, uint8_t reg, uint8_t *buf,
				 uint32_t size)
{
	return i2c_burst_read_dt(&bus->i2c, reg, buf, size);
}

static int icm456xx_reg_write_i2c(const union icm456xx_bus *bus, uint8_t reg, uint8_t *buf,
				  uint32_t size)
{
	return i2c_burst_write_dt(&bus->i2c, reg, buf, size);
}

const struct icm456xx_bus_io icm456xx_bus_io_i2c = {
	.check = icm456xx_bus_check_i2c,
	.read = icm456xx_reg_read_i2c,
	.write = icm456xx_reg_write_i2c,
};
