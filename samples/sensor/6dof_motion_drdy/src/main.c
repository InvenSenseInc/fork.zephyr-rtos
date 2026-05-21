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

#include "invn_algo.h"

static struct sensor_trigger data_trigger;

uint64_t sample_time=0;

/* Flag set from IMU device irq handler */
static volatile int irq_from_device;

/*
 * Get a device structure from a devicetree node from alias
 * "6dof_motion_drdy0".
 */
static const struct device *get_6dof_motion_device(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(6dof_motion_drdy0));

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

static void handle_6dof_motion_drdy(const struct device *dev, const struct sensor_trigger *trig)
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
			sample_time = (k_uptime_ticks()*1000000)/CONFIG_SYS_CLOCK_TICKS_PER_SEC;			
		}
	}
}

int main(void)
{
	const struct device *dev = get_6dof_motion_device();
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value temperature;
	
    int16_t acc_raw[3];
    int16_t gyr_raw[3];
    int16_t mag_raw[3];
    uint8_t accurracy[3];
    float quat[4];

	struct sensor_value sample_rate;
	if (dev == NULL) {
		return 0;
	}

	data_trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	if (sensor_trigger_set(dev, &data_trigger, handle_6dof_motion_drdy) < 0) {
		printf("Cannot configure data trigger!!!\n");
		return 0;
	}
	sample_rate.val1 = 25;
    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sample_rate);
    sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sample_rate);
	invn_algo_init(10);

	k_sleep(K_MSEC(1000));

	while (1) {

		if (irq_from_device) {
			sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
			sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
			sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);

			irq_from_device = 0;
            acc_raw[0] = -(int16_t) ((sensor_value_to_double(&accel[0])*32768/8)/9.80665);
            acc_raw[1] = -(int16_t) ((sensor_value_to_double(&accel[2])*32768/8)/9.80665);
            acc_raw[2] = -(int16_t) ((sensor_value_to_double(&accel[1])*32768/8)/9.80665);
            gyr_raw[0] = -(int16_t) ((sensor_value_to_double(&gyro[0])*32768*180/2000)/3.14159);
            gyr_raw[1] = -(int16_t) ((sensor_value_to_double(&gyro[2])*32768*180/2000)/3.14159);
            gyr_raw[2] = -(int16_t) ((sensor_value_to_double(&gyro[1])*32768*180/2000)/3.14159);
            mag_raw[0] = (int16_t) 0;//(mag_temp_data.x*32768/2000);
            mag_raw[1] = -(int16_t) 0;//(mag_temp_data.z*32768/2000);
            mag_raw[2] = (int16_t) 0;//(mag_temp_data.y*32768/2000);
            invn_algo_process((int64_t) sample_time, acc_raw, gyr_raw, mag_raw, quat, accurracy);
			printf("%lld: temp %.2f Cel "
			       "  accel %f %f %f m/s/s "
			       "  gyro  %f %f %f rad/s\n"
			       "  quat  %f %f %f %f\n",
			       sample_time, sensor_value_to_double(&temperature),
			       sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
			       sensor_value_to_double(&accel[2]), sensor_value_to_double(&gyro[0]),
			       sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]),
			       (double) quat[0], (double) quat[1],(double) quat[2],(double) quat[3]);
		}
	}
	return 0;
}
