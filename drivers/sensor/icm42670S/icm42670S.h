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

typedef int (*icm42670S_bus_check_fn)(const union icm42670S_bus *bus);
typedef int (*icm42670S_reg_read_fn)(const union icm42670S_bus *bus,
				  uint8_t start, uint8_t *buf, int size);
typedef int (*icm42670S_reg_write_fn)(const union icm42670S_bus *bus,
				   uint8_t reg, uint8_t val);

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



#endif /* ZEPHYR_DRIVERS_SENSOR_ICM42670S_ICM42670S_H_ */
