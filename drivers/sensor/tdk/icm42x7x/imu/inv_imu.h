/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _INV_IMU_H_
#define _INV_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup IMU IMU
 *  @brief Describes IMU
 *  @{
 */

/** @file inv_imu.h */

/* Device description */

#ifdef CONFIG_ICM42670P

#define INV_IMU_STRING_ID "ICM-42670-P"
#define INV_IMU_WHOAMI 0x67
#define INV_IMU_REV INV_IMU_REV_A
#define INV_IMU_IS_GYRO_SUPPORTED 1

#elif CONFIG_ICM42670S

#define INV_IMU_STRING_ID "ICM-42670-S"
#define INV_IMU_WHOAMI 0x69
#define INV_IMU_REV INV_IMU_REV_A
#define INV_IMU_IS_GYRO_SUPPORTED 1

#elif  CONFIG_ICM42671P

#define INV_IMU_STRING_ID "ICM-42671-P"
#define INV_IMU_WHOAMI 0x52
#define INV_IMU_REV INV_IMU_REV_B
#define INV_IMU_IS_GYRO_SUPPORTED 1

#elif  CONFIG_ICM42671S

#define INV_IMU_STRING_ID "ICM-42671-S"
#define INV_IMU_WHOAMI 0x54
#define INV_IMU_REV INV_IMU_REV_B
#define INV_IMU_IS_GYRO_SUPPORTED 1

#elif  CONFIG_ICM42370P

#define INV_IMU_STRING_ID "ICM-42370-P"
#define INV_IMU_WHOAMI 0x0D
#define INV_IMU_REV INV_IMU_REV_A
#define INV_IMU_IS_GYRO_SUPPORTED 0

#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_H_ */

/** @} */
