/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/logging/log.h>
#include <math.h>

static struct sensor_trigger irq_trigger;

LOG_MODULE_REGISTER(ICP201XX_SAMPLE, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Get a device structure from a devicetree node with compatible
 * "invensense,icm101xx". (If there are multiple, just pick one.)
 */
static const struct device *get_icp201xx_device(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(invensense_icp201xx);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

static void handle_icp201xx(const struct device *dev, const struct sensor_trigger *trig)
{

	if (trig->type == SENSOR_TRIG_DELTA) {

		LOG_INF("PRESSURE CHANGE INTERRUPT\n");
		k_sleep(K_MSEC(500));
	}
}

int main(void)
{
	const struct device *dev = get_icp201xx_device();
	struct sensor_value val;

	if (dev == NULL) {
		return 0;
	}
	irq_trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_DELTA,
		.chan = SENSOR_CHAN_PRESS,
	};

	sensor_value_from_float(&val, 0.01);
	sensor_attr_set(dev, SENSOR_CHAN_PRESS, SENSOR_ATTR_SLOPE_TH, &val);

	if (sensor_trigger_set(dev, &irq_trigger, handle_icp201xx) < 0) {
		printf("Cannot configure threshold trigger!!!\n");
		return 0;
	}
	LOG_INF("Starting ICP201xx pressure change interrupt sample.\n");

	return 0;
}
