/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor/icm42670.h>

#include "icm42670.h"
#include "invn_algo_aml.h"

LOG_MODULE_REGISTER(ICM42670S_AML, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_ICM42670S_AML
/*
 * ICM mounting matrix for AML referential.
 * For AML algorithm, accelerometer data are inverted compared to ICM convention,
 * It expects gravity minus acceleration and ICM measures acceleration minus gravity
 * Refer to AML documentation
 */
static int32_t accel_mounting_matrix[9] = {0, -1, 0, -1, 0, 0, 0, 0, 1};

static int32_t gyro_mounting_matrix[9] = {0, 1, 0, 1, 0, 0, 0, 0, -1};

int icm42670S_aml_init(const struct device *dev, inv_imu_device_t *s, int8_t delta_gain_x,
		       int8_t delta_gain_y)
{
	int rc = 0;
	/* Contains algorithms configuration */
	InvnAlgoAMLConfig config;
	struct icm42670_data *data = dev->data;

	/* FSR */
	config.acc_fsr = 16;
	config.gyr_fsr = 2000;
	data->accel_fs = 16;
	data->gyro_fs = 2000;

	/* Delta Gain */
	config.delta_gain[0] = delta_gain_x;
	config.delta_gain[1] = delta_gain_y;

	config.gestures_auto_reset = 1;

	/* Initialize AML algorithms */
	rc = invn_algo_aml_init(s, &config);
	if (rc != 0) {
		LOG_ERR("Error. Authentication Failed.");
		return rc;
	} else {
		LOG_DBG("Algorithm init OK");
	}

	/* Configure ICM device */
	rc |= inv_imu_set_accel_fsr(s, ACCEL_CONFIG0_FS_SEL_16g);
	rc |= inv_imu_set_gyro_fsr(s, GYRO_CONFIG0_FS_SEL_2000dps);

	/* Set frequencies */
	rc |= inv_imu_set_accel_frequency(s, ACCEL_CONFIG0_ODR_100_HZ);
	rc |= inv_imu_set_gyro_frequency(s, GYRO_CONFIG0_ODR_100_HZ);

	/* Enable sensors */
	rc |= inv_imu_enable_accel_low_noise_mode(s);
	rc |= inv_imu_enable_gyro_low_noise_mode(s);

	if (rc != 0) {
		LOG_ERR("Error while configuring ICM device");
	}

	return rc;
}

static void apply_mounting_matrix(const int32_t matrix[9], int16_t raw[3])
{
	unsigned int i;
	int16_t out_raw[3];

	for (i = 0; i < 3; i++) {
		out_raw[i] = ((int16_t)matrix[3 * i + 0] * raw[0]);
		out_raw[i] += ((int16_t)matrix[3 * i + 1] * raw[1]);
		out_raw[i] += ((int16_t)matrix[3 * i + 2] * raw[2]);
	}

	raw[0] = out_raw[0];
	raw[1] = out_raw[1];
	raw[2] = out_raw[2];
}

void icm42670S_aml_process(const struct device *dev)
{
	struct icm42670_data *data = dev->data;
	static InvnAlgoAMLInput input;
	static InvnAlgoAMLOutput output;

	input.racc_data[0] = data->accel_x;
	input.racc_data[1] = data->accel_y;
	input.racc_data[2] = data->accel_z;

	input.rgyr_data[0] = data->gyro_x;
	input.rgyr_data[1] = data->gyro_y;
	input.rgyr_data[2] = data->gyro_z;

	apply_mounting_matrix(accel_mounting_matrix, input.racc_data);
	apply_mounting_matrix(gyro_mounting_matrix, input.rgyr_data);

	/* Process the AML algo */
	invn_algo_aml_process(&input, &output);

	if (output.status & INVN_ALGO_AML_STATUS_DELTA_COMPUTED) {
		data->delta[0] = output.delta[0];
		data->delta[1] = output.delta[1];

		data->quaternion[0] = output.quaternion[0];
		data->quaternion[1] = output.quaternion[1];
		data->quaternion[2] = output.quaternion[2];
		data->quaternion[3] = output.quaternion[3];
	}

	if (output.status & INVN_ALGO_AML_STATUS_STATIC) {
		data->remote_static = 1;
	} else {
		data->remote_static = 0;
	}

	data->swipes_detected = 0;
	if (output.swipes_detected != 0) {
		data->swipes_detected = output.swipes_detected;
		output.swipes_detected = 0;
	}

	if ((uint8_t)output.remote_position != data->remote_position) {
		data->remote_position = (uint8_t)output.remote_position;
	}

	if (output.status & INVN_ALGO_AML_STATUS_NEW_GYRO_OFFSET) {
		data->gyro_offset[0] = output.gyr_offset[0];
		data->gyro_offset[1] = output.gyr_offset[1];
		data->gyro_offset[2] = output.gyr_offset[2];
	}
}

void icm42670S_aml_quaternion_convert(struct sensor_value *val, int32_t raw_val)
{
	int64_t conv_val;

	/* Converting q10 */
	conv_val = (int64_t)(raw_val * 1000000) / (1 << 10);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

#endif
