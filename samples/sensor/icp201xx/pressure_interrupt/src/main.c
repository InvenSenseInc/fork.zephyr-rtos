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

	if (trig->type == SENSOR_TRIG_THRESHOLD) {

		LOG_INF("THRESHOLD INTERRUPT");
		k_sleep(K_SEC(1));

	}
}


#define ATMOSPHERICAL_PRESSURE_KPA 101.325
#define TO_KELVIN(temp_C) (273.15 + temp_C)
#define HEIGHT_TO_PRESSURE_COEFF 0.03424 // M*g/R = (0,0289644 * 9,80665 / 8,31432)

#define PRESSURE_TO_HEIGHT_COEFF 29.27127 // R / (M*g) = 8,31432 / (0,0289644 * 9,80665)
#define LOG_ATMOSPHERICAL_PRESSURE 4.61833 // ln(101.325)

static float convertToPressure(float height_m, float temperature_C)
{
    return ATMOSPHERICAL_PRESSURE_KPA * exp(-HEIGHT_TO_PRESSURE_COEFF*height_m/TO_KELVIN(temperature_C));
}
int main(void)
{
	const struct device *dev = get_icp201xx_device();
	float pressure_threshold, altitude, temperature;
	struct sensor_value val;

	if (dev == NULL) {
		return 0;
	}
	data_trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_THRESHOLD,
		.chan = SENSOR_CHAN_ALL,
	};
			k_sleep(K_USEC(40000));

	/* Read current altitude and temperature */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	sensor_channel_get(dev, SENSOR_CHAN_ALTITUDE,&val);
	printf("val1=%d val2=%d\n",val.val1,val.val2);
	altitude = sensor_value_to_float(&val);
	sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,&val);
	temperature = sensor_value_to_float(&val);

	printf("altitude %f m\n",altitude);
	printf("temperature %f m\n",temperature);
	
	pressure_threshold = convertToPressure(altitude+0.5, temperature);
	printf("threshold %f m\n",pressure_threshold);
	sensor_value_from_float(&val, pressure_threshold);
	sensor_attr_set(dev,SENSOR_CHAN_PRESS,SENSOR_ATTR_LOWER_THRESH,&val);
	
	if (sensor_trigger_set(dev, &data_trigger, handle_icp201xx) < 0) {
		printf("Cannot configure threshold trigger!!!\n");
		return 0;
	}
	LOG_INF("Starting ICP201xx threshold interrupt sample.\n");
	
	return 0;
}
