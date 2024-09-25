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

LOG_MODULE_REGISTER(ICT1531X_SAMPLE, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Get a device structure from a devicetree node with compatible
 * "invensense,ict1531x". (If there are multiple, just pick one.)
 */
static const struct device *get_ict1531x_device(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(invensense_ict1531x);

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

int main(void)
{
	const struct device *dev = get_ict1531x_device();

	if (dev == NULL) {
		return 0;
	}
	struct sensor_value mag[3];
	struct sensor_value temperature;

	LOG_INF("Starting ICT1531x sample.\n");

	while (1) {
		sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
		sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);

		LOG_INF("temp=%.2f Cel, mag_x=%0.2f Gs, mag_y=%0.2f Gs, mag_z=%0.2f Gs",
			sensor_value_to_double(&temperature), sensor_value_to_double(&mag[0]),
			sensor_value_to_double(&mag[1]),sensor_value_to_double(&mag[2]));
	}
	return 0;
}
