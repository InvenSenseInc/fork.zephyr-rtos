/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_ICM456XX_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_ICM456XX_H_

#include <zephyr/drivers/sensor.h>

/**
 * @file
 * @brief Extended public API for ICM456XX MEMS sensor
 *
 * Some capabilities and operational requirements for this sensor
 * cannot be expressed within the sensor driver abstraction.
 */

/** ICM456XX power mode */
#define ICM456XX_LOW_NOISE_MODE (0)
#define ICM456XX_LOW_POWER_MODE (1)

/**
 * @brief Extended sensor attributes for ICM456XX MEMS sensor
 *
 * This exposes attributes for the ICM456XX which can be used for
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
enum sensor_attribute_icm456xx {
	/** BW filtering */

	/** Low-pass filter configuration */
	SENSOR_ATTR_BW_FILTER_LPF = SENSOR_ATTR_PRIV_START,
	/** Averaging configuration */
	SENSOR_ATTR_AVERAGING,
};
#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_ICM456XX_H_ */
