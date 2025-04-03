/*
 * Copyright (c) 2025 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "icm456xx.h"
#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_driver_advanced.h"

int icm456xx_apex_enable(inv_imu_device_t *s)
{
	int                            rc = 0;
	inv_imu_int_state_t            int_config;
	inv_imu_edmp_apex_parameters_t apex_parameters;
	inv_imu_edmp_int_state_t       apex_int_config;

	rc |= inv_imu_get_config_int(s, INV_IMU_INT1, &int_config);
	int_config.INV_FIFO_THS = INV_IMU_DISABLE;
	int_config.INV_UI_DRDY = INV_IMU_DISABLE;
	int_config.INV_EDMP_EVENT = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(s, INV_IMU_INT1, &int_config);

	/* Set EDMP ODR */
	rc |= inv_imu_edmp_set_frequency(s, DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ);

	/* Set ODR */
	rc |= inv_imu_set_accel_frequency(s, ACCEL_CONFIG0_ACCEL_ODR_50_HZ);

	/* Set BW = ODR/4 */
	rc |= inv_imu_set_accel_ln_bw(s, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);

	/* Select WUOSC clock to have accel in ULP (lowest power mode) */
	rc |= inv_imu_select_accel_lp_clk(s, SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC);

	/* Set AVG to 1x */
	rc |= inv_imu_set_accel_lp_avg(s, IPREG_SYS2_REG_129_ACCEL_LP_AVG_1);

	/* Ensure all DMP features are disabled before running init procedure */
	rc |= inv_imu_edmp_disable_pedometer(s);
	rc |= inv_imu_edmp_disable_tilt(s);
	rc |= inv_imu_edmp_disable(s);

	/* Request DMP to re-initialize APEX */
	rc |= inv_imu_edmp_recompute_apex_decimation(s);

	/* Configure APEX parameters */
	rc |= inv_imu_edmp_get_apex_parameters(s, &apex_parameters);
	apex_parameters.power_save_en = INV_IMU_DISABLE;
	rc |= inv_imu_adv_disable_wom(s);
	rc |= inv_imu_edmp_set_apex_parameters(s, &apex_parameters);

	/* Set accel in low-power mode if ODR slower than 800 Hz, otherwise in low-noise mode */
	rc |= inv_imu_set_accel_mode(s, PWR_MGMT0_ACCEL_MODE_LP);

	/* Wait for accel startup time */
	k_msleep(10);

	/* Disable all APEX interrupt and enable only the one we need */
	memset(&apex_int_config, INV_IMU_DISABLE, sizeof(apex_int_config));

	/* Apply interrupt configuration */
	rc |= inv_imu_edmp_set_config_int_apex(s, &apex_int_config);

	return rc;
}

int icm456xx_apex_fetch_from_dmp(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	int rc = 0;
	inv_imu_int_state_t      int_state;
	inv_imu_edmp_int_state_t apex_state = { 0 };

	/* Read APEX interrupt status */
	rc |= inv_imu_get_int_status(&data->driver, INV_IMU_INT1, &int_state);

	/* Test Pedometer interrupt */
	if (int_state.INV_EDMP_EVENT) {
		uint8_t step_cnt_ovflw = 0;
		/* Read APEX interrupt status */
		rc |= inv_imu_edmp_get_int_apex_status(&data->driver, &apex_state);

		/* Pedometer */
		if (apex_state.INV_STEP_CNT_OVFL) {
			step_cnt_ovflw = 1;
		}

		if ( apex_state.INV_STEP_DET) {
			inv_imu_edmp_pedometer_data_t ped_data;
			rc |= inv_imu_edmp_get_pedometer_data(&data->driver, &ped_data);
			if (rc == INV_IMU_OK) {
				data->pedometer_cnt = (unsigned long)ped_data.step_cnt + (step_cnt_ovflw * UINT16_MAX);
				data->pedometer_activity = ped_data.activity_class;
				data->pedometer_cadence = ped_data.step_cadence;
			}
		}

		/* SMD */
		if (apex_state.INV_SMD)
			data->apex_status = ICM456XX_APEX_STATUS_MASK_SMD;

		/* Tilt */
		if (apex_state.INV_TILT_DET) {
			data->apex_status = ICM456XX_APEX_STATUS_MASK_TILT;
		}
	}

	return rc;
}

void icm456xx_apex_pedometer_cadence_convert(struct sensor_value *val, uint8_t raw_val,
		uint8_t dmp_odr_hz)
{
	int64_t conv_val;

	/* Converting u6.2 */
	conv_val = (int64_t)(dmp_odr_hz << 2) * 1000000 / (raw_val + (raw_val & 0x03));
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

int icm456xx_apex_enable_pedometer(const struct device *dev, inv_imu_device_t *s)
{
	int rc = 0;
	inv_imu_edmp_int_state_t       apex_int_config;

	rc = inv_imu_edmp_get_config_int_apex(s, &apex_int_config);
	apex_int_config.INV_STEP_CNT_OVFL = INV_IMU_ENABLE;
	apex_int_config.INV_STEP_DET      = INV_IMU_ENABLE;
	/* Apply interrupt configuration */
	rc |= inv_imu_edmp_set_config_int_apex(s, &apex_int_config);
	rc |= inv_imu_edmp_enable_pedometer(s);
	/* Enable EDMP if at least one feature is enabled */
	rc |= inv_imu_edmp_enable(s);
	return rc;
}

int icm456xx_apex_enable_tilt(inv_imu_device_t *s)
{
	int rc = 0;
	inv_imu_edmp_int_state_t       apex_int_config;

	rc = inv_imu_edmp_get_config_int_apex(s, &apex_int_config);
	apex_int_config.INV_TILT_DET = INV_IMU_ENABLE;
	/* Apply interrupt configuration */
	rc |= inv_imu_edmp_set_config_int_apex(s, &apex_int_config);
	rc |= inv_imu_edmp_enable_tilt(s);
	/* Enable EDMP if at least one feature is enabled */
	rc |= inv_imu_edmp_enable(s);
	return rc;
}

int icm456xx_apex_enable_smd(inv_imu_device_t *s)
{
	int rc = 0;
	inv_imu_edmp_int_state_t       apex_int_config;

	rc = inv_imu_edmp_get_config_int_apex(s, &apex_int_config);
	apex_int_config.INV_SMD = INV_IMU_ENABLE;
	/* Apply interrupt configuration */
	rc |= inv_imu_edmp_set_config_int_apex(s, &apex_int_config);
	rc |= inv_imu_edmp_enable_smd(s);
	/* Enable EDMP if at least one feature is enabled */
	rc |= inv_imu_edmp_enable(s);
	return rc;
}

int icm456xx_apex_enable_wom(inv_imu_device_t *s)
{
	int rc = 0;
	rc |= inv_imu_adv_configure_wom(s, DEFAULT_WOM_THS_MG, DEFAULT_WOM_THS_MG,
			DEFAULT_WOM_THS_MG, TMST_WOM_CONFIG_WOM_INT_MODE_ORED,
			TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
	rc |= inv_imu_adv_enable_wom(s);

	return rc;
}
