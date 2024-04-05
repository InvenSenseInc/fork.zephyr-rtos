/*
 * $License:
 * Copyright (C) 2018 InvenSense Corporation, All Rights Reserved.
 * $
 */

#ifndef _INVN_ALGO_AML_H_
#define _INVN_ALGO_AML_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup AML AML
 *  \brief AIR MOTION Library provides pointing from 6-axis sensor and additionally swipe motion
 * recognition. \warning Supported sampling frequency is [100 Hz]
 * *  @section INVN_ALGO_AML_INTRO Introduction
 *    The Air Motion Library is a highly programmable API for managing mouse cursor motion from 3
 * axes gyroscope and 3 axes accelerometer. Embedded TDK proprietary algorithms are responsible for
 * converting motion sensors data into delta X and delta Y pointer movements. The library is
 * intended to be used in free space pointing devices to operate in-air point and click navigation,
 * just like a classic 2D mouse will do on a desk. In addition to its pointing feature, Air Motion
 * Library is also capable of swipe motion recognition: 'Up', 'Down', 'Left', 'Right', 'Clockwise'
 * and 'CounterClockwise' swipes are currently supported.
 *
 *  @section INVN_ALGO_AML_REF AIR MOTION Library sensors referential
 *      @li When the device lies flat on its back, accelerometer must see +1G on Z axis
 *      @li When the device lies on its right side, accelerometer must see +1G on Y axis
 *      @li When the device is held vertically pointing down, accelerometer must see +1G on X axis
 *      @li A clockwise rotation around the gyroscope X axis generates positive values
 *      @li A clockwise rotation around the gyroscope Y axis generates positive values i.e. negative
 * delta Y on screen (cursor moves to the top)
 *      @li A clockwise rotation around the gyroscope Z axis generates positive values i.e. positive
 * delta X on screen (cursor moves to the right)
 *
 *  @image html axis.png
 *  @image latex axis.png
 *
 *  @section AML_API API
 *    The AIR MOTION Library uses the following functions :
 *      @li invn_algo_aml_init() : This function must be called for library initialization, each
 * time processing (re)starts.
 *      @li invn_algo_aml_process() : This function must be called each time new sensors values are
 * available. The library is designed to run at 100Hz.
 *      @li invn_algo_aml_reset_swipe_recognition() : This function must be called to trigger each
 * new swipe gesture detection.
 *
 *  @section AML_CALIBRATION Calibration
 *    Gyroscope offsets values can fluctuate over time depending on various parameters.
 *    The library supports a continuous calibration which computes gyroscope offsets values in real
 * time.
 *
 *    Each time the invn_algo_aml_process() is called, gyroscope instant "staticness" is checked.
 *    If the device is considered static, new gyroscope offset values are computed. The remote can
 * then be considered as calibrated. You can check 'InvnAlgoAMLOutput.status' to track calibration
 * status over time.
 *
 *    'InvnAlgoAMLConfig.gyr_fsr' parameter is used to calibrate motion algorithms.
 *    'InvnAlgoAMLConfig.acc_fsr' parameter is used for roll-compensation feature.
 *
 *  @section AML_ROLLCOMPENSATION RollCompensation
 *    The Air Motion Library embeds a roll-compensation algorithm along with its pointing feature.
 *    Roll-compensation allows user's movement to be reliably reproduced on the screen,
 * independently from the device roll orientation.
 *
 *  @section AML_EASYCLICK EasyClick
 *    This feature allows user to perform more accurate and stable mouse clicks for a better
 * experience. On click press, the library will freeze the pointer (dX and dY null) to avoid
 * undesired movement. During this period, if the device movement quantity is too important, the
 * pointer will be released sooner.
 *
 */

/* Define default algorithm init values */
#define INVN_ALGO_AML_DELTA_GAIN_DEFAULT 15

/* Define bit mask for output status */
#define INVN_ALGO_AML_STATUS_DELTA_COMPUTED         0x1
#define INVN_ALGO_AML_STATUS_STATIC                 0x2
#define INVN_ALGO_AML_STATUS_NEW_GYRO_OFFSET        0x4
#define INVN_ALGO_AML_STATUS_REMOTE_POSITION_CHANGE 0x8

/* Define for different swipe detected */
#define INVN_ALGO_AML_SWIPE_LEFT             0x1
#define INVN_ALGO_AML_SWIPE_RIGTH            0x2
#define INVN_ALGO_AML_SWIPE_UP               0x4
#define INVN_ALGO_AML_SWIPE_DOWN             0x8
#define INVN_ALGO_AML_SWIPE_CLOCKWISE        0x10
#define INVN_ALGO_AML_SWIPE_COUNTERCLOCKWISE 0x20

/* Define for different remote position */
typedef enum {
	INVN_ALGO_AML_TOP,    /*Z Down */
	INVN_ALGO_AML_BOTTOM, /*Z Up */
	INVN_ALGO_AML_LEFT,   /*Y Up */
	INVN_ALGO_AML_RIGHT,  /*Y Down */
	INVN_ALGO_AML_FRONT,  /*X Up */
	INVN_ALGO_AML_REAR    /*X Down */
} RemotePosition;

/* Forward declarations */
struct inv_imu_device;

/*! \struct InvnAlgoAMLInput
 * AML input structure (6-axis raw data sensor and click button) \ingroup AML
 */
typedef struct {
	int16_t racc_data[3]; /*!<  3-axis raw accelerometer data */
	int16_t rgyr_data[3]; /*!<  3-axis raw gyroscope data */
	uint8_t click_button; /*!<  Click button state */
} InvnAlgoAMLInput;

/*! \struct InvnAlgoAMLOutput
 * AML output structure (calibrated sensor offsets, pointing output, swipe gestures detected and
 * quaternion)  \ingroup AML
 */
typedef struct {
	uint8_t status;                 /*!< Mask to specify status outputs */
	int16_t gyr_offset[3];          /*!< Computed 3-axis raw gyroscope offsets [q4]*/
	int8_t delta[2];                /*!< Computed delta values for pointing */
	uint8_t swipes_detected;        /*!< Bitfield of detected swipes */
	int32_t quaternion[4];          /*!< Computed W, X, Y and Z quaternion coefficients [q10] */
	RemotePosition remote_position; /*!< Orientation of remote */
} InvnAlgoAMLOutput;

/*! \struct InvnAlgoAMLConfig
 * AML configuration structure (gain for X and Y axis, sensor offsets, sensor full scale range)
 * \ingroup AML
 */
typedef struct {
	int8_t delta_gain[2];        /*!< Delta gain values for X and Y axes */
	int16_t gyr_offset[3];       /*!< Initial 3-axis raw gyroscope offsets [q4] */
	uint16_t acc_fsr;            /*!< Accelerometer full scale range [g] */
	uint16_t gyr_fsr;            /*!< Gyroscope full scale range [dps] */
	uint8_t gestures_auto_reset; /*!< En/disable gestures auto reset */
} InvnAlgoAMLConfig;

/*!
\brief Return library version x.y.z-suffix as a char array
 * \retval library version a char array "x.y.z-suffix"
 * \ingroup AML
 */
const char *invn_algo_aml_version(void);

/*!
\brief Initializes algorithms with default parameters and reset states.
 * \param[in] icm_device Invensense IMU device pointer.
 * \param[in] config algo configuration structure in InvnAlgoAMLConfig.
 * \return initialization success indicator.
 * \retval 0 Success
 * \retval 1 Fail
 * \ingroup AML
 */
uint8_t invn_algo_aml_init(struct inv_imu_device *icm_device, const InvnAlgoAMLConfig *config);

/*!
\brief Performs algorithm computation.
 * \param[in] inputs algorithm input in InvnAlgoAMLInput.
 * \param[out] outputs algorithm output. Output status reports updated outputs in InvnAlgoAMLOutput.
 * \ingroup AML
 */
void invn_algo_aml_process(const InvnAlgoAMLInput *inputs, InvnAlgoAMLOutput *outputs);

/*!
\brief Performs reset of swipes gesture recognition. Allows to restart gesture processing without
resetting pointing.
 * \ingroup AML
 */
void invn_algo_aml_reset_swipe_recognition(void);

#ifdef __cplusplus
}
#endif

#endif
