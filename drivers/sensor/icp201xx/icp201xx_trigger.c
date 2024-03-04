/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include "icp201xx.h"

LOG_MODULE_DECLARE(ICP201XX, CONFIG_SENSOR_LOG_LEVEL);



static void icp201xx_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb, uint32_t pins)
{
	icp201xx_data *drv_data =
		CONTAINER_OF(cb, icp201xx_data, gpio_cb);

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure_dt(drv_data->gpio_int_p, GPIO_INT_DISABLE);

	k_sem_give(&drv_data->gpio_sem);

}

static void icp201xx_thread_cb(const struct device *dev)
{
	icp201xx_data *drv_data = dev->data;
	const struct icp201xx_config *cfg = dev->config;
	uint8_t i_status;

	if (drv_data->irq_handler != NULL) {
		drv_data->irq_handler(dev,
					     drv_data->irq_trigger);
	}
	inv_icp201xx_get_int_status(&drv_data->icp_device,&i_status);
	if ( i_status )
		inv_icp201xx_clear_int_status(&drv_data->icp_device,i_status);
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_LEVEL_LOW);
}

static void icp201xx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	struct device* dev = (struct device*)p1;
	icp201xx_data *drv_data = (icp201xx_data *)dev->data;

	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		icp201xx_thread_cb(dev);
	}
}

int icp201xx_init_interrupt(const struct device *dev)
{
	icp201xx_data *drv_data = dev->data;
	const struct icp201xx_config *cfg = dev->config;
	int result = 0;

	if (!gpio_is_ready_dt(&cfg->gpio_int)) {
		LOG_ERR("gpio_int gpio not ready");
		return -ENODEV;
	}
	
	drv_data->gpio_int_p = &cfg->gpio_int;

	gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);
	gpio_init_callback(&drv_data->gpio_cb, icp201xx_gpio_callback, BIT(cfg->gpio_int.pin));
	result = gpio_add_callback(cfg->gpio_int.port, &drv_data->gpio_cb);

	if (result < 0) {
		LOG_ERR("Failed to set gpio callback");
		return result;
	}

	k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_ICP201XX_THREAD_STACK_SIZE, icp201xx_thread, (struct device *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ICP201XX_THREAD_PRIORITY), 0, K_NO_WAIT);


	return 0;
}

int icp201xx_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	int rc = 0;
	icp201xx_data *drv_data = dev->data;
	const struct icp201xx_config *cfg = dev->config;

	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_DISABLE);

	if (handler == NULL) {
		return -1;
	}

	if ((trig->type != SENSOR_TRIG_DATA_READY)&&(trig->type != SENSOR_TRIG_DELTA)&&(trig->type != SENSOR_TRIG_THRESHOLD))
	{
		return -ENOTSUP;
	}
	drv_data->irq_handler = handler;
	drv_data->irq_trigger = trig;
	
	rc |= icp201xx_init_interrupt(dev);
	if (rc < 0) {
		LOG_ERR("Failed to initialize interrupt.");
		return rc;
	}
	if (trig->type == SENSOR_TRIG_DATA_READY) {
		rc = icp201xx_fifo_interrupt(dev,1);
	} else if (trig->type == SENSOR_TRIG_DELTA) {
		rc = icp201xx_pressure_change_interrupt(dev,drv_data->pressure_change);
	} else if (trig->type == SENSOR_TRIG_THRESHOLD) {
		rc = icp201xx_pressure_interrupt(dev,drv_data->pressure_threshold);
	}

	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_LEVEL_LOW);

	return 0;
}

