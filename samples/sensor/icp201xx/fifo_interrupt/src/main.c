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

static struct sensor_trigger data_trigger;

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

	if (trig->type == SENSOR_TRIG_DATA_READY) {
		int rc = sensor_sample_fetch_chan(dev, trig->chan);

		if (rc < 0) {
			printf("sample fetch failed: %d\n", rc);
			printf("cancelling trigger\n");
			(void)sensor_trigger_set(dev, trig, NULL);
			return;
		} else if (rc == 0) {

			struct sensor_value pressure;
			struct sensor_value temperature;
			struct sensor_value altitude;

			sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure);
			sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
			sensor_channel_get(dev, SENSOR_CHAN_ALTITUDE, &altitude);

			LOG_INF("temp %.2f Cel, pressure %f kPa, altitude %f m",
				sensor_value_to_double(&temperature),
				sensor_value_to_double(&pressure),
				sensor_value_to_double(&altitude));
		}
	}
}

int main(void)
{
	const struct device *dev = get_icp201xx_device();

	if (dev == NULL) {
		return 0;
	}
	data_trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	if (sensor_trigger_set(dev, &data_trigger, handle_icp201xx) < 0) {
		printf("Cannot configure data trigger!!!\n");
		return 0;
	}
	LOG_INF("Starting ICP201xx fifo interrupt sample.\n");

	return 0;
}
