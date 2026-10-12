/*
 * Copyright (c) 2023 Google LLC
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/sys/check.h>

#include "ictx53xx.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ICTX53XX_ASYNC, CONFIG_SENSOR_LOG_LEVEL);

static void ictx53xx_complete_result(struct rtio *ctx, const struct rtio_sqe *sqe, int result,
				     void *arg)
{
	ARG_UNUSED(result);
	struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)sqe->userdata;
	struct rtio_cqe *cqe;
	int err = 0;

	do {
		cqe = rtio_cqe_consume(ctx);
		if (cqe != NULL) {
			err = cqe->result;
			rtio_cqe_release(ctx, cqe);
		}
	} while (cqe != NULL);

	if (err != 0) {
		rtio_iodev_sqe_err(iodev_sqe, err);
	} else {
		rtio_iodev_sqe_ok(iodev_sqe, 0);
	}
}

int ictx53xx_encode(const struct device *dev, const struct sensor_read_config *read_config,
		    uint8_t trigger_status, uint8_t *buf)
{
	struct ictx53xx_encoded_data *edata = (struct ictx53xx_encoded_data *)buf;
	uint64_t cycles;
	int err;

	ARG_UNUSED(dev);
	ARG_UNUSED(trigger_status);

	/* Get timestamp */
	err = sensor_clock_get_cycles(&cycles);
	if (err != 0) {
		return err;
	}

	edata->header.timestamp = sensor_clock_cycles_to_ns(cycles);

	return 0;
}


static void ictx53xx_submit_one_shot(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	uint32_t min_buf_len = sizeof(struct ictx53xx_encoded_data);
	int err;
	uint8_t *buf;
	uint32_t buf_len;
	struct ictx53xx_encoded_data *edata;
	struct ictx53xx_data *data = (struct ictx53xx_data *)dev->data;

	err = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (err < 0 || buf_len < min_buf_len || !buf) {
		LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
		rtio_iodev_sqe_err(iodev_sqe, err);
		return;
	}

	edata = (struct ictx53xx_encoded_data *)buf;

	err = ictx53xx_encode(dev, cfg, 0, buf);
	if (err != 0) {
		LOG_ERR("Failed to encode sensor data");
		rtio_iodev_sqe_err(iodev_sqe, err);
		return;
	}

	struct rtio_sqe *read_sqe;

	err = ictx53xx_i2c_prep_reg_read_rtio_async(&data->bus, INV_ICT_TEMP_DATA_LSB, edata->readings,
					   sizeof(edata->readings), &read_sqe);
	if (err < 0) {
		LOG_ERR("Failed to prepare async read operation");
		rtio_iodev_sqe_err(iodev_sqe, err);
		return;
	}
	read_sqe->flags |= RTIO_SQE_CHAINED;

	struct rtio_sqe *complete_sqe = rtio_sqe_acquire(data->bus.rtio_ctx);

	if (!complete_sqe) {
		LOG_ERR("Failed to acquire completion SQE");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		rtio_sqe_drop_all(data->bus.rtio_ctx);
		return;
	}

	rtio_sqe_prep_callback_no_cqe(complete_sqe, &ictx53xx_complete_result, (void *)dev,
				      iodev_sqe);

	rtio_submit(data->bus.rtio_ctx, 0);
}

void ictx53xx_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	ictx53xx_submit_one_shot(dev, iodev_sqe);
}
