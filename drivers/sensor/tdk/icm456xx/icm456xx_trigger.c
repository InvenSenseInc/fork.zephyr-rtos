/*
 * Copyright (c) 2025 TDK Invensense
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2016 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include "icm456xx.h"
#include "icm456xx_trigger.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(ICM456XX, CONFIG_SENSOR_LOG_LEVEL);

static void icm456xx_gpio_callback(const struct device *dev, struct gpio_callback *cb,
				   uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	struct icm456xx_data *data = CONTAINER_OF(cb, struct icm456xx_data, gpio_cb);

#if defined(CONFIG_ICM456XX_TRIGGER_OWN_THREAD)
	k_sem_give(&data->gpio_sem);
#elif defined(CONFIG_ICM456XX_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}

static void icm456xx_thread_cb(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	const struct icm456xx_config *cfg = dev->config;

	icm456xx_lock(dev);
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_DISABLE);

	if (data->data_ready_handler) {
		data->data_ready_handler(dev, data->data_ready_trigger);
	}

	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
	icm456xx_unlock(dev);
}

#if defined(CONFIG_ICM456XX_TRIGGER_OWN_THREAD)

static void icm456xx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct icm456xx_data *data = p1;

	while (1) {
		k_sem_take(&data->gpio_sem, K_FOREVER);
		icm456xx_thread_cb(data->dev);
	}
}
#elif defined(CONFIG_ICM456XX_TRIGGER_GLOBAL_THREAD)

static void icm456xx_work_handler(struct k_work *work)
{
	struct icm456xx_data *data = CONTAINER_OF(work, struct icm456xx_data, work);

	icm456xx_thread_cb(data->dev);
}

#endif

int icm456xx_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	struct icm456xx_data *data = dev->data;
	const struct icm456xx_config *cfg = dev->config;

	if (!handler) {
		return -EINVAL;
	}

	icm456xx_lock(dev);
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_DISABLE);

	if (trig->type == SENSOR_TRIG_DATA_READY) {
		data->data_ready_handler = handler;
		data->data_ready_trigger = trig;
#ifdef CONFIG_TDK_APEX
	} else if (trig->type == SENSOR_TRIG_MOTION) {
		data->data_ready_handler = handler;
		data->data_ready_trigger = trig;
#endif
	} else {
		return -ENOTSUP;
	}

	icm456xx_unlock(dev);
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}

int icm456xx_trigger_init(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	const struct icm456xx_config *cfg = dev->config;
	int res = 0;

	if (!gpio_is_ready_dt(&cfg->gpio_int)) {
		LOG_ERR("gpio_int gpio not ready");
		return -ENODEV;
	}

	data->dev = dev;
	gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);
	gpio_init_callback(&data->gpio_cb, icm456xx_gpio_callback, BIT(cfg->gpio_int.pin));
	res = gpio_add_callback(cfg->gpio_int.port, &data->gpio_cb);

	if (res < 0) {
		LOG_ERR("Failed to set gpio callback");
		return res;
	}

	k_mutex_init(&data->mutex);

#if defined(CONFIG_ICM456XX_TRIGGER_OWN_THREAD)
	k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);
	k_thread_create(&data->thread, data->thread_stack, CONFIG_ICM456XX_THREAD_STACK_SIZE,
			icm456xx_thread, data, NULL, NULL,
			K_PRIO_COOP(CONFIG_ICM456XX_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_ICM456XX_TRIGGER_GLOBAL_THREAD)
	data->work.handler = icm456xx_work_handler;
#endif

	return gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_INACTIVE);
}

int icm456xx_trigger_enable_interrupt(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	int err = 0;
	inv_imu_int_pin_config_t int_pin_config;

	/*
	 * Configure interrupts pins
	 * - Polarity High
	 * - Pulse mode
	 * - Push-Pull drive
	 */
	int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
	int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
	err = inv_imu_set_pin_config_int(&data->driver, INV_IMU_INT1, &int_pin_config);

	return err;
}

void icm456xx_lock(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;

	k_mutex_lock(&data->mutex, K_FOREVER);
}

void icm456xx_unlock(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;

	k_mutex_unlock(&data->mutex);
}
