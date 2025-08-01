/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM45686_STREAM_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM45686_STREAM_H_

int icm456xx_stream_init(const struct device *dev);

void icm456xx_stream_submit(const struct device *dev,
			    struct rtio_iodev_sqe *iodev_sqe);

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM45686_STREAM_H_ */
