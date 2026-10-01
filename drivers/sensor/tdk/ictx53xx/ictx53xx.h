/*
 * Copyright (c) 2025 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICTX53XX_H_
#define ZEPHYR_DRIVERS_SENSOR_ICTX53XX_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>
#include "ict/inv_ict_driver.h"

/* Address value has a read bit */
#define REG_READ_BIT BIT(7)

struct ictx53xx_bus {
	/* for communication to the bus controller */
	struct rtio *rtio_ctx;
	struct rtio_iodev *iodev;
};

struct ictx53xx_config {
	inv_ict_mode_t op_mode;
};

struct ictx53xx_data {
	uint16_t x;
	uint16_t y;
	uint16_t z;
	int16_t temp;
	inv_ict_device_t driver;
#ifdef CONFIG_SENSOR_ASYNC_API
	struct ictx53xx_async_fetch_ctx {
		struct rtio_iodev_sqe *iodev_sqe;
		uint64_t timestamp;
		struct k_work_delayable async_fetch_work;
	} work_ctx;
#endif
	struct ictx53xx_bus bus;
};

int ictx53xx_prep_reg_read_rtio_async(const struct ictx53xx_bus *bus, uint8_t reg, uint8_t *buf,
				      size_t size, struct rtio_sqe **out);

int ictx53xx_prep_reg_write_rtio_async(const struct ictx53xx_bus *bus, uint8_t reg, const uint8_t *buf,
				      size_t size, struct rtio_sqe **out);

int ictx53xx_reg_read_rtio(const struct ictx53xx_bus *bus, uint8_t start, uint8_t *buf, int size);

int ictx53xx_reg_write_rtio(const struct ictx53xx_bus *bus, uint8_t reg, const uint8_t *buf, int size);

#endif /* ZEPHYR_DRIVERS_SENSOR_ICTX53XX_H_*/
