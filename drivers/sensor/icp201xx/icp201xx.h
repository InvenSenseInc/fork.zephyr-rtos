/*
 * Copyright (c) 2023 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICP201XXX_ICP101XXX_H_
#define ZEPHYR_DRIVERS_SENSOR_ICP201XXX_ICP101XXX_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include "ICP201xx/Icp201xx.h"
#include "ICP201xx/Icp201xxSerif.h"

#define DT_DRV_COMPAT invensense_icp201xx

#define ICP201XX_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#define ICP201XX_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

typedef struct {
	int32_t raw_pressure;
	int32_t raw_temperature;
	icp201xx_op_mode_t op_mode;
	float pressure_change;
	float pressure_threshold;
	inv_icp201xx_t icp_device;
	struct gpio_callback gpio_cb;
	const struct gpio_dt_spec *gpio_int_p;

	const struct sensor_trigger *irq_trigger;
	sensor_trigger_handler_t irq_handler;

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ICP201XX_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
} icp201xx_data;

struct icp201xx_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec gpio_int;
	icp201xx_op_mode_t op_mode;
};

int icp201xx_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);

int icp201xx_fifo_interrupt(const struct device *dev, uint8_t fifo_watermark);
int icp201xx_pressure_interrupt(const struct device *dev, float pressure);
int icp201xx_pressure_change_interrupt(const struct device *dev, float pressure_delta);

#endif
