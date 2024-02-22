/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/icm42670S.h>
#include <stdio.h>

static struct sensor_trigger data_trigger;

/* Flag set from IMU device irq handler */
static volatile int irq_from_device = 0;

/*
 * Get a device structure from a devicetree node with compatible
 * "invensense,icm42670S". (If there are multiple, just pick one.)
 */
static const struct device *get_icm42670S_device(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(invensense_icm42670s);

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

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u", h, min, s, ms);
	return buf;
}

static void handle_icm42670S_drdy(const struct device *dev, const struct sensor_trigger *trig)
{
	if (trig->type == SENSOR_TRIG_DATA_READY) {
		int rc = sensor_sample_fetch_chan(dev, trig->chan);

		if (rc < 0) {
			printf("sample fetch failed: %d\n", rc);
			printf("cancelling trigger due to failure: %d\n", rc);
			(void)sensor_trigger_set(dev, trig, NULL);
			return;
		} else if (rc == 0) {
			irq_from_device = 1;
		}
	}
}

int main(void)
{
	const struct device *dev = get_icm42670S_device();
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value temperature;

	if (dev == NULL) {
		return 0;
	}

	data_trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	if (sensor_trigger_set(dev, &data_trigger, handle_icm42670S_drdy) < 0) {
		printf("Cannot configure data trigger!!!\n");
		return 0;
	}

	k_sleep(K_MSEC(1000));

	while (1) {

		if (irq_from_device) {
			sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
			sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
			sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);

			printf("[%s]: temp %.2f Cel "
			       "  accel %f %f %f m/s/s "
			       "  gyro  %f %f %f rad/s\n",
			       now_str(), sensor_value_to_double(&temperature),
			       sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
			       sensor_value_to_double(&accel[2]), sensor_value_to_double(&gyro[0]),
			       sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]));
			irq_from_device = 0;
		}
	}
	return 0;
}
