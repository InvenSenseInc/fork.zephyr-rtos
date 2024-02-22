/*
 * Copyright (c) 2023 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for ICM42670S accessed via SPI.
 */

#include <zephyr/logging/log.h>
#include "icm42670S.h"

#if ICM42670S_BUS_SPI

LOG_MODULE_DECLARE(ICM42670S, CONFIG_SENSOR_LOG_LEVEL);

static int icm42670S_bus_check_spi(const union icm42670S_bus *bus)
{
	return spi_is_ready_dt(&bus->spi) ? 0 : -ENODEV;
}

static int icm42670S_reg_read_spi(const union icm42670S_bus *bus, uint8_t start, uint8_t *buf,
				  int size)
{
	uint8_t addr;
	const struct spi_buf tx_buf = {.buf = &addr, .len = 1};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf[2];
	const struct spi_buf_set rx = {.buffers = rx_buf, .count = ARRAY_SIZE(rx_buf)};
	int i;

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;

	rx_buf[1].len = 1;

	for (i = 0; i < size; i++) {
		int ret;

		addr = (start + i) | 0x80;
		rx_buf[1].buf = &buf[i];

		ret = spi_transceive_dt(&bus->spi, &tx, &rx);
		if (ret) {
			LOG_DBG("spi_transceive FAIL %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int icm42670S_reg_write_spi(const union icm42670S_bus *bus, uint8_t reg, uint8_t val)
{
	uint8_t cmd[] = {reg & 0x7F, val};
	const struct spi_buf tx_buf = {.buf = cmd, .len = sizeof(cmd)};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	int ret;

	ret = spi_write_dt(&bus->spi, &tx);
	if (ret) {
		LOG_DBG("spi_write FAIL %d\n", ret);
		return ret;
	}
	return 0;
}

const struct icm42670S_bus_io icm42670S_bus_io_spi = {
	.check = icm42670S_bus_check_spi,
	.read = icm42670S_reg_read_spi,
	.write = icm42670S_reg_write_spi,
};
#endif /* ICM42670S_BUS_SPI */
