/*
 * Copyright (c) 2023 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_ICM42670S_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_ICM42670S_H_

#include <zephyr/drivers/sensor.h>


/**
 * @file
 * @brief Extended public API for ICM42670S 6-axis MEMS sensor
 *
 * Some capabilities and operational requirements for this sensor
 * cannot be expressed within the sensor driver abstraction.
 */

/** ICM42670S power mode */
#define ICM42670S_POWER_OFF         (0)
#define ICM42670S_LOW_POWER_MODE    (1)
#define ICM42670S_LOW_NOISE_MODE    (2)

/** ICM42670S APEX features */
#define ICM42670S_APEX_PEDOMETER    (1)
#define ICM42670S_APEX_TILT         (2)
#define ICM42670S_APEX_SMD          (3)
#define ICM42670S_APEX_WOM          (4)

/** ICM42670S swipes gesture */
#define ICM42670S_AML_SWIPE_LEFT                0x1
#define ICM42670S_AML_SWIPE_RIGTH               0x2
#define ICM42670S_AML_SWIPE_UP                  0x4
#define ICM42670S_AML_SWIPE_DOWN                0x8
#define ICM42670S_AML_SWIPE_CLOCKWISE           0x10
#define ICM42670S_AML_SWIPE_COUNTERCLOCKWISE    0x20

/** ICM42670S remote position */
#define ICM42670S_AML_TOP      (0)
#define ICM42670S_AML_BOTTOM   (1)
#define ICM42670S_AML_LEFT     (2)
#define ICM42670S_AML_RIGHT    (3)
#define ICM42670S_AML_FRONT    (4)
#define ICM42670S_AML_REAR     (5)

/**
 * @brief Extended sensor attributes for ICM42670S 6-axis MEMS sensor
 *
 * This exposes attributes for the ICM42670S which can be used for
 * setting the signal path filtering parameters.
 *
 * The signal path starts with ADCs for the gyroscope and accelerometer. 
 * Low-Noise Mode and Low-Power Mode options are available for the 
 * accelerometer. Only Low-Noise Mode is available for gyroscope.
 * In Low-Noise Mode, the ADC output is sent through an Anti-Alias Filter
 * (AAF). The AAF is a filter with fixed coefficients (not user configurable),
 * also the AAF cannot be bypassed. The AAF is followed by a 1st Order Low Pass 
 * Filter (LPF) with user selectable filter bandwidth options. 
 * In Low-Power Mode, the accelerometer ADC output is sent through an Average
 * filter, with user configurable average filter setting.
 * The output of 1st Order LPF in Low-Noise Mode, or Average filter in Low-Power
 * Mode is subject to ODR selection, with user selectable ODR.
 */
enum sensor_attribute_icm42670S {
	/** BW filtering */	 
	
	/** Low-pass filter configuration */
	SENSOR_ATTR_BW_FILTER_LPF = SENSOR_ATTR_PRIV_START,
	/** Averaging configuration */
	SENSOR_ATTR_AVERAGING,
};


/**
 * @brief Extended sensor channel for ICM42670S 6-axis MEMS sensor
 *
 * This exposes sensor channel for the ICM42670S which can be used for
 * getting the APEX features data.
 * 
 * The APEX (Advanced Pedometer and Event Detection – neXt gen) features of 
 * ICM-42670-S consist of:
 * ** Pedometer: Tracks step count.
 * ** Tilt Detection: Detect the Tilt angle exceeds 35 degrees. 
 * ** Wake on Motion (WoM): Detects motion when accelerometer samples exceed 
 * a programmable threshold. This motion event can be used to enable device 
 * operation from sleep mode.
 * ** Significant Motion Detector (SMD): Detects significant motion based on
 * accelerometer data.
 *
 * This also exposes sensor channel for the ICM42670S which can be used for
 * getting the Air Motion Library (AML) data.
 *
 * The AML library provides pointing from ICM42670S 6-axis sensor and intended 
 * to be used in free space pointing devices to operate in-air point and click
 * navigation, just like a classic 2D mouse will do on a desk. The library 
 * additionally provides swipe motion recognition, gyroscope biases calibration
 * and quaternion orientation.
 */
enum sensor_channel_icm42670S {
	
	/** APEX features */
	SENSOR_CHAN_APEX_MOTION = SENSOR_CHAN_PRIV_START,
	
	/** AML */
	SENSOR_CHAN_AML,
	SENSOR_CHAN_AML_OUTPUT_DELTA_POINTING,
	SENSOR_CHAN_AML_OUTPUT_GESTURES,
	SENSOR_CHAN_AML_OUTPUT_GYRO_CALIBRATTION,
	SENSOR_CHAN_AML_OUTPUT_QUATERNION,
};
#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_ICM42670S_H_ */
