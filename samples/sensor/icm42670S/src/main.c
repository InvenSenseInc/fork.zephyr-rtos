/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

struct sensor_value accel[3];
struct sensor_value gyro[3];
struct sensor_value temperature;

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

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}

static int process_icm42670S(const struct device *dev)
{
	int rc = sensor_sample_fetch(dev);
	
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					gyro);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP,
					&temperature);
	}
	
	if (rc != 0) { 
		printf("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}

static struct sensor_trigger data_trigger;
static struct sensor_trigger tap_trigger;
static struct sensor_trigger double_tap_trigger;

static void handle_icm42670S_drdy(const struct device *dev,
				 const struct sensor_trigger *trig)
{
	int rc = process_icm42670S(dev);

	if (rc != 0) {
		printf("cancelling trigger due to failure: %d\n", rc);
		(void)sensor_trigger_set(dev, trig, NULL);
		return;
	}
}

static void handle_icm42670S_tap(const struct device *dev,
				const struct sensor_trigger *trig)
{
	printf("Tap Detected!\n");
}

static void handle_icm42670S_double_tap(const struct device *dev,
				       const struct sensor_trigger *trig)
{
	printf("Double Tap detected!\n");
}

int main(void)
{
	const struct device *dev = get_icm42670S_device();

	if (dev == NULL) {
		return 0;
	}

	tap_trigger = (struct sensor_trigger) {
		.type = SENSOR_TRIG_TAP,
		.chan = SENSOR_CHAN_ALL,
	};

	if (sensor_trigger_set(dev, &tap_trigger,
			       handle_icm42670S_tap) < 0) {
		printf("Cannot configure tap trigger!!!\n");
		return 0;
	}

	double_tap_trigger = (struct sensor_trigger) {
		.type = SENSOR_TRIG_DOUBLE_TAP,
		.chan = SENSOR_CHAN_ALL,
	};

	if (sensor_trigger_set(dev, &double_tap_trigger,
			       handle_icm42670S_double_tap) < 0) {
		printf("Cannot configure double tap trigger!!!\n");
		return 0;
	}

	data_trigger = (struct sensor_trigger) {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};

	if (sensor_trigger_set(dev, &data_trigger,
			       handle_icm42670S_drdy) < 0) {
		printf("Cannot configure data trigger!!!\n");
		return 0;
	}

	printf("Configured for triggered sampling.\n");
	
	while (1) {
		
		printf("[%s]: ACC m/s/s: %d.%06d %d.%06d %d.%06d "
		             "GYR rad/s: %d.%06d %d.%06d %d.%06d "
					 "TEMP degC: %d.%06d\n",
			   now_str(),
		       accel[0].val1, accel[0].val2,
		       accel[1].val1, accel[1].val2,
		       accel[2].val1, accel[2].val2,
		       gyro[0].val1, gyro[0].val2,
		       gyro[1].val1, gyro[1].val2,
		       gyro[2].val1, gyro[2].val2,
			   temperature.val1, temperature.val2);
	}
}
