/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/icm42670.h>
#include <stdio.h>

static struct sensor_trigger data_trigger;

/* Flag set from IMU device irq handler */
static volatile int irq_from_device = 0;

/*
 * Get a device structure from a devicetree node with compatible
 * "invensense,icm42670". (If there are multiple, just pick one.)
 */
static const struct device *get_icm42670_device(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(invensense_icm42670);

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

#if defined(CONFIG_ICM42670S_AML_POINTING) || defined(CONFIG_ICM42670S_AML_GESTURES) ||            \
	defined(CONFIG_ICM42670S_AML_GYR_OFFSET) || defined(CONFIG_ICM42670S_AML_QUATERNION)
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
#endif

static void handle_icm42670_drdy(const struct device *dev, const struct sensor_trigger *trig)
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
	const struct device *dev = get_icm42670_device();
	struct sensor_value aml_config[2];

	if (dev == NULL) {
		return 0;
	}

	aml_config[0].val1 = 15; /* Delta Gain X */
	aml_config[1].val1 = 15; /* Delta Gain Y */
	sensor_attr_set(dev, SENSOR_CHAN_AML, SENSOR_ATTR_CONFIGURATION, aml_config);

	data_trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_AML,
	};
	if (sensor_trigger_set(dev, &data_trigger, handle_icm42670_drdy) < 0) {
		printf("Cannot configure data trigger!!!\n");
		return 0;
	}

	k_sleep(K_MSEC(1000));

	while (1) {

		if (irq_from_device) {
#ifdef CONFIG_ICM42670S_AML_POINTING
			struct sensor_value pointing_data[2];
			sensor_channel_get(dev, SENSOR_CHAN_AML_OUTPUT_DELTA_POINTING,
					   pointing_data);

			printf("[%s]: AML Pointing Delta=[%d, %d]\n", now_str(),
			       pointing_data[0].val1, pointing_data[1].val1);
#endif
#ifdef CONFIG_ICM42670S_AML_GESTURES
			struct sensor_value aml_data[3];
			static uint8_t previous_position = 0, previous_status = 0;
			sensor_channel_get(dev, SENSOR_CHAN_AML_OUTPUT_GESTURES, aml_data);

			if (aml_data[0].val1 & ICM42670S_AML_SWIPE_LEFT) {
				printf("[%s]: AML Gestures: Swipe Left \n", now_str());
			}
			if (aml_data[0].val1 & ICM42670S_AML_SWIPE_RIGTH) {
				printf("[%s]: AML Gestures: Swipe Rigth \n", now_str());
			}
			if (aml_data[0].val1 & ICM42670S_AML_SWIPE_UP) {
				printf("[%s]: AML Gestures: Swipe Up \n", now_str());
			}
			if (aml_data[0].val1 & ICM42670S_AML_SWIPE_DOWN) {
				printf("[%s]: AML Gestures: Swipe Down \n", now_str());
			}
			if (aml_data[0].val1 & ICM42670S_AML_SWIPE_CLOCKWISE) {
				printf("[%s]: AML Gestures: Swipe Clockwise \n", now_str());
			}
			if (aml_data[0].val1 & ICM42670S_AML_SWIPE_COUNTERCLOCKWISE) {
				printf("[%s]: AML Gestures: Swipe Counter Clockwise \n", now_str());
			}

			if (aml_data[1].val1 != previous_position) {
				printf("[%s]: AML Remote position: %s \n", now_str(),
				       aml_data[1].val1 == ICM42670S_AML_TOP      ? "Top"
				       : aml_data[1].val1 == ICM42670S_AML_BOTTOM ? "Bottom"
				       : aml_data[1].val1 == ICM42670S_AML_LEFT   ? "Left"
				       : aml_data[1].val1 == ICM42670S_AML_RIGHT  ? "Right"
				       : aml_data[1].val1 == ICM42670S_AML_FRONT  ? "Front"
				       : aml_data[1].val1 == ICM42670S_AML_REAR   ? "Rear"
										  : "Unknown");
				previous_position = aml_data[1].val1;
			}

			if (aml_data[2].val1 != previous_status) {
				printf("[%s]: AML Remote status: %s \n", now_str(),
				       aml_data[2].val1 == 1 ? "Static" : "Non-static");
				previous_status = aml_data[2].val1;
			}
#endif
#ifdef CONFIG_ICM42670S_AML_GYR_OFFSET
			struct sensor_value gyr_offset[3];
			static struct sensor_value previous_gyr_offset[3];
			sensor_channel_get(dev, SENSOR_CHAN_AML_OUTPUT_GYRO_CALIBRATTION,
					   gyr_offset);

			if (memcmp(gyr_offset, previous_gyr_offset, sizeof(struct sensor_value)) !=
			    0) {
				printf("[%s]: AML Gyro biases: %f %f %f rad/s\n", now_str(),
				       sensor_value_to_double(&gyr_offset[0]),
				       sensor_value_to_double(&gyr_offset[1]),
				       sensor_value_to_double(&gyr_offset[2]));
				memcpy(previous_gyr_offset, gyr_offset,
				       sizeof(struct sensor_value));
			}
#endif
#ifdef CONFIG_ICM42670S_AML_QUATERNION
			struct sensor_value quaternion[4];
			sensor_channel_get(dev, SENSOR_CHAN_AML_OUTPUT_QUATERNION, quaternion);

			printf("[%s]:AML Quaternion=[%f %f %f %f]\n", now_str(),
			       sensor_value_to_double(&quaternion[0]),
			       sensor_value_to_double(&quaternion[1]),
			       sensor_value_to_double(&quaternion[2]),
			       sensor_value_to_double(&quaternion[3]));
#endif
			irq_from_device = 0;
		}
	}
	return 0;
}
