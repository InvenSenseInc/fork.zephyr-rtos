/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

#ifdef CONFIG_ICM45686_STREAM
#define NUM_SENSORS 1

#define ICM45686_TRIGGERS                                   \
        {SENSOR_TRIG_FIFO_FULL, SENSOR_STREAM_DATA_INCLUDE}, \
        {SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE}

SENSOR_DT_STREAM_IODEV(
    icm45686_iodev,
    DT_ALIAS(tdk_apex_sensor0),
    ICM45686_TRIGGERS
);
#endif

#ifdef CONFIG_ICM45686_TRIGGER
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

static int process_icm45686(const struct device *dev)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];

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
		printf("[%s]\n"
		       "  accel % f % f % f m/s/s\n"
		       "  gyro  % f % f % f rad/s\n",
		       now_str(),
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
                       sensor_value_to_double(&gyro[2]));

	} else {
		printf("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}

static struct sensor_trigger data_trigger;

static void handle_icm45686_drdy(const struct device *dev,
				 const struct sensor_trigger *trig)
{
	int rc = process_icm45686(dev);

	if (rc != 0) {
		printf("cancelling trigger due to failure: %d\n", rc);
		(void)sensor_trigger_set(dev, trig, NULL);
		return;
	}
}
#endif

#ifdef CONFIG_ICM45686_STREAM
RTIO_DEFINE_WITH_MEMPOOL(accel_ctx, NUM_SENSORS, NUM_SENSORS,
                         NUM_SENSORS * 20, 256, sizeof(void *));

static int print_accels_stream(const struct device *dev, struct rtio_iodev *iodev)
{
        int rc = 0;
        struct sensor_three_axis_data accel_data = {0};
        const struct sensor_decoder_api *decoder;
        struct rtio_cqe *cqe;
        uint8_t *buf;
        uint32_t buf_len;
        struct rtio_sqe *handles[NUM_SENSORS];

        /* Start the streams */
        for (int i = 0; i < NUM_SENSORS; i++) {
                printk("sensor_stream\n");
                sensor_stream(iodev, &accel_ctx, NULL, &handles[i]);
        }

        while (1) {
                cqe = rtio_cqe_consume_block(&accel_ctx);

                if (cqe->result != 0) {
                        printk("async read failed %d\n", cqe->result);
                        return cqe->result;
                }
                rc = rtio_cqe_get_mempool_buffer(&accel_ctx, cqe, &buf, &buf_len);

                if (rc != 0) {
                        printk("get mempool buffer failed %d\n", rc);
                        return rc;
                }

                const struct device *sensor = dev;

                rtio_cqe_release(&accel_ctx, cqe);

		                rc = sensor_get_decoder(sensor, &decoder);

                if (rc != 0) {
                        printk("sensor_get_decoder failed %d\n", rc);
                        return rc;
                }

                /* Frame iterator values when data comes from a FIFO */
                uint32_t accel_fit = 0;

                /* Number of accelerometer data frames */
                uint16_t frame_count;

                rc = decoder->get_frame_count(buf,
                                (struct sensor_chan_spec) {SENSOR_CHAN_ACCEL_XYZ, 0}, &frame_count);

                if (rc != 0) {
                        printk("sensor_get_decoder failed %d\n", rc);
                        return rc;
                }
                /* If a tap has occurred lets print it out */
                if (decoder->has_trigger(buf, SENSOR_TRIG_TAP)) {
                        printk("Tap! Sensor %s\n", dev->name);
                }

                /* Decode all available accelerometer sample frames */
                for (int i = 0; i < frame_count; i++) {
                        decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_ACCEL_XYZ, 0},
                                        &accel_fit, 1, &accel_data);

                        printk("Accel data for %s (%" PRIq(6) ", %" PRIq(6)
                                        ", %" PRIq(6) ") %lluns\n", dev->name,
                        PRIq_arg(accel_data.readings[0].x, 6, accel_data.shift),
                        PRIq_arg(accel_data.readings[0].y, 6, accel_data.shift),
                        PRIq_arg(accel_data.readings[0].z, 6, accel_data.shift),
                        (accel_data.header.base_timestamp_ns
                        + accel_data.readings[0].timestamp_delta));
                }

                rtio_release_buffer(&accel_ctx, buf, buf_len);
        }

        return rc;
}
#endif

enum icm45686_bus_type {
        ICM45686_BUS_SPI,
        ICM45686_BUS_I2C,
        ICM45686_BUS_I3C,
};

int main(void)
{
	printk("0");
	const struct device *const icm45686 = DEVICE_DT_GET_ONE(invensense_icm45686);
        struct rtio_iodev *iodev = &icm45686_iodev; 

	if (!device_is_ready(icm45686)) {
		printk("sensor icm45686: device not ready.\n");
		return 0;
	}
#ifdef CONFIG_ICM45686_TRIGGER
	data_trigger = (struct sensor_trigger) {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};

	if (sensor_trigger_set(icm45686, &data_trigger,
				handle_icm45686_drdy) < 0) {
		printf("Cannot configure data trigger!!!\n");
		return 0;
	}

#endif

#ifdef CONFIG_ICM45686_STREAM
	int ret= 0;

	while (1) {
		ret = print_accels_stream(icm45686, iodev);
		if (ret < 0) {
			printk("Error while execute print_accels_Stream");
			return 0;
		}
		k_msleep(1000);
	}
#endif
	return 0;
}
