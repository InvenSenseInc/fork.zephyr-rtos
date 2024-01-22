/*
 * Copyright (c) 2023 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "icm42670S.h"
#include "imu/inv_imu_apex.h"

#if defined(CONFIG_ICM42670S_APEX_PEDOMETER) || defined(CONFIG_ICM42670S_APEX_TILT)
int icm42670S_apex_enable(inv_imu_device_t *s)
{
	int err = 0;
	inv_imu_apex_parameters_t apex_inputs;
	inv_imu_interrupt_parameter_t config_int = { (inv_imu_interrupt_value)0 };
	
	/* Disabling FIFO to avoid extra power consumption due to ALP config */
	err |= inv_imu_configure_fifo(s, INV_IMU_FIFO_DISABLED);
	
	/* Disable data ready interrupt and enable Pedometer interrupts */
	err |= inv_imu_get_config_int1(s, &config_int);
	config_int.INV_UI_DRDY       = INV_IMU_DISABLE;
	config_int.INV_STEP_DET      = INV_IMU_ENABLE;
	config_int.INV_STEP_CNT_OVFL = INV_IMU_ENABLE;
	config_int.INV_TILT_DET      = INV_IMU_ENABLE;
	err |= inv_imu_set_config_int1(s, &config_int);

	/* Enable accelerometer to feed the APEX Pedometer algorithm */
	err |= inv_imu_set_accel_frequency(s, ACCEL_CONFIG0_ODR_50_HZ);

	/* Set 2x averaging, in order to minimize power consumption (16x by default) */
	err |= inv_imu_set_accel_lp_avg(s, ACCEL_CONFIG1_ACCEL_FILT_AVG_2);
	err |= inv_imu_enable_accel_low_power_mode(s);

	/* Get the default parameters for the APEX features */
	err |= inv_imu_apex_init_parameters_struct(s, &apex_inputs);

	/* Configure the power mode Normal mode.
	*  Avalaible mode : Low Power mode (WoM+Pedometer), 
	*  configure the WoM to wake-up the DMP once it goes in power save mode 
	*/
	apex_inputs.power_save = APEX_CONFIG0_DMP_POWER_SAVE_DIS;
	err |= inv_imu_apex_configure_parameters(s, &apex_inputs);

	/* Configure sampling frequency to 50Hz */
	err |= inv_imu_apex_set_frequency(s, APEX_CONFIG1_DMP_ODR_50Hz);

	return err;
}
#endif

#ifdef CONFIG_ICM42670S_APEX_PEDOMETER
int icm42670S_apex_enable_pedometer(const struct device *dev, inv_imu_device_t *s)
{
	struct icm42670S_data *data = dev->data;
	
	data->dmp_odr_hz = 50;
	/* Enable the pedometer */	
	return inv_imu_apex_enable_pedometer(s);
}

int icm42670S_apex_pedometer_fetch_from_dmp(const struct device *dev)
{
	struct icm42670S_data *data = dev->data;
	int rc = 0;
	uint8_t int_status3;
	
	/* Read Pedometer interrupt status */
	rc |= inv_imu_read_reg(&data->driver, INT_STATUS3, 1, &int_status3);

	if (int_status3 & (INT_STATUS3_STEP_DET_INT_MASK)) {
		inv_imu_apex_step_activity_t apex_pedometer;
		uint8_t step_cnt_ovflw = 0;
			
		if (int_status3 & INT_STATUS3_STEP_CNT_OVF_INT_MASK)
			step_cnt_ovflw = 1;

		rc |= inv_imu_apex_get_data_activity(&data->driver, &apex_pedometer);
		
		if (data->pedometer_cnt != apex_pedometer.step_cnt + step_cnt_ovflw * (uint64_t)UINT16_MAX) {
			data->pedometer_cnt = apex_pedometer.step_cnt + step_cnt_ovflw * (uint64_t)UINT16_MAX;
			data->pedometer_activity = apex_pedometer.activity_class;
			data->pedometer_cadence = apex_pedometer.step_cadence;
		} else {
			/* Pedometer data processing */
			rc = 1;
		}
	}
	return rc;
}

void icm42670S_apex_pedometer_cadence_convert(struct sensor_value *val, uint8_t raw_val, uint8_t dmp_odr_hz)
{
	int64_t conv_val;

	/* Converting u6.2 */
	conv_val = (int64_t)(dmp_odr_hz << 2) * 1000000 / (raw_val + (raw_val & 0x03));
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}
#endif

#ifdef CONFIG_ICM42670S_APEX_TILT
int icm42670S_apex_enable_tilt(inv_imu_device_t *s)
{
	/* Enable the pedometer */	
	return inv_imu_apex_enable_tilt(s);
}

int icm42670S_apex_tilt_fetch_from_dmp(const struct device *dev)
{
	struct icm42670S_data *data = dev->data;
	int rc = 0;
	uint8_t int_status3;
	
	/* Read Pedometer interrupt status */
	rc |= inv_imu_read_reg(&data->driver, INT_STATUS3, 1, &int_status3);

	if (int_status3 & (INT_STATUS3_TILT_DET_INT_MASK))
		return rc;
	else 
		return -1;
}
#endif
