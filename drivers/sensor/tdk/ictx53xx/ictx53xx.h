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

/* Measurement time */
#define ICTX53XX_MEASUREMENT_TIME_US        3050
#define ICTX53XX_MEASUREMENT_EXTRA_TIME_US  51
struct ictx53xx_bus {
	/* for communication to the bus controller */
	struct rtio *rtio_ctx;
	struct rtio_iodev *iodev;
};

struct ictx53xx_config {
	uint32_t sens;
};

struct ictx53xx_data {
	uint16_t x;
	uint16_t y;
	uint16_t z;
	int16_t temp;
	inv_ict_device_t driver;
	inv_ict_mode_t mode;
	inv_ict_odr_t odr;
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
static inline uint8_t ictx53xx_hz_to_reg(const struct sensor_value *val)
{
	if (val->val1 >= 320) {
		return INV_ICT_MODE_CTRL_REG_ODR_320_HZ;
	} else if (val->val1 >= 200) {
		return INV_ICT_MODE_CTRL_REG_ODR_200_HZ;
	} else if (val->val1 >= 100) {
		return INV_ICT_MODE_CTRL_REG_ODR_100_HZ;
	} else if (val->val1 >= 50) {
		return INV_ICT_MODE_CTRL_REG_ODR_50_HZ;
	} else if (val->val1 >= 20) {
		return INV_ICT_MODE_CTRL_REG_ODR_20_HZ;
	} else if (val->val1 >= 10) {
		return INV_ICT_MODE_CTRL_REG_ODR_10_HZ;
	} else if (val->val1 > 0) {
		return INV_ICT_MODE_CTRL_REG_ODR_5_HZ;
	} else {
		return 0;
	}
}

int ictx53xx_reg_read_rtio(const struct ictx53xx_bus *bus, uint8_t start, uint8_t *buf, int size);
static inline void ictx53xx_reg_to_hz(uint8_t reg, struct sensor_value *val)
{
	val->val1 = 0;
	val->val2 = 0;
	switch (reg) {
	case INV_ICT_MODE_CTRL_REG_ODR_320_HZ:
		val->val1 = 320;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_200_HZ:
		val->val1 = 200;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_100_HZ:
		val->val1 = 100;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_50_HZ:
		val->val1 = 50;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_20_HZ:
		val->val1 = 20;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_10_HZ:
		val->val1 = 10;
		break;
	case INV_ICT_MODE_CTRL_REG_ODR_5_HZ:
		val->val1 = 5;
		break;
	}
}

int ictx53xx_reg_write_rtio(const struct ictx53xx_bus *bus, uint8_t reg, const uint8_t *buf, int size);

#endif /* ZEPHYR_DRIVERS_SENSOR_ICTX53XX_H_*/
