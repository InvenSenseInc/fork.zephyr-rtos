/*
$License:
	Copyright (C) 2025 InvenSense Corporation, All Rights Reserved.
$
*/
 

#ifndef _INVN_ALGO_INTERFACECOMMON_H_
#define _INVN_ALGO_INTERFACECOMMON_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define INVN_ALGO_GAF_INPUT_MASK_ACC 1 ///< Raw Accel input mask
#define INVN_ALGO_GAF_INPUT_MASK_GYR 2 ///< Raw Gyro input mask
#define INVN_ALGO_GAF_INPUT_MASK_MAG 4 ///< Raw Mag input mask
#define INVN_ALGO_GAF_INPUT_MASK_GYR_CAL 8 ///< Cal Gyro input mask
#define INVN_ALGO_GAF_INPUT_MASK_ACC_CAL 16 ///< Cal Acc input mask
#define INVN_ALGO_GAF_INPUT_MASK_MAG_CAL 32 ///< Cal Mag input mask

#define INVN_ALGO_GAF_OUTPUT_MASK_DATAPROCESS 1 ///< DATAPROCESS output mask indicates that at least the library  did some process on a valid input data
#define INVN_ALGO_GAF_OUTPUT_MASK_GYRO_CAL    8  ///< Gyro calibration output update mask
#define INVN_ALGO_GAF_OUTPUT_MASK_ACCEL_CAL   16  ///< Accel calibration output update mask
#define INVN_ALGO_GAF_OUTPUT_MASK_MAG_CAL     32  ///< Mag calibration output update mask
#define INVN_ALGO_GAF_OUTPUT_MASK_QUAT_AG     64  ///< Game Rotation Vector (Accel and Gyro Fusion) output update mask 
#define INVN_ALGO_GAF_OUTPUT_MASK_QUAT_AGM    128 ///< Rotation Vector (Accel, Gyro and Mag Fusion) output update mask 
#define INVN_ALGO_GAF_OUTPUT_MASK_GRAVITY     256 ///< Gravity vector output update mask
#define INVN_ALGO_GAF_OUTPUT_MASK_LINEARACC   512 ///< Linear acceleration vector output update mask
#define INVN_ALGO_GAF_OUTPUT_MASK_QUAT_AM     1024 ///< Geomag Rotation Vector (Accel and Mag Fusion) output update mask



#ifdef __cplusplus
}
#endif

#endif
