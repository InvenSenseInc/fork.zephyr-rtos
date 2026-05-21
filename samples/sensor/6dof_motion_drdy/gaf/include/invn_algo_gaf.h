/*
$License:
	Copyright (C) 2025 InvenSense Corporation, All Rights Reserved.
$
*/
 

#ifndef _INVN_ALGO_GAF_H_
#define _INVN_ALGO_GAF_H_

#include <stdint.h>
#include "invn_algo_interface_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INVN_ALGO_GAF_DATA_STRUCTURE_SIZE 3088

#define ERROR_INVN_ALGO_GAF_STRUCT_NULL_POINTER 127

/** @defgroup GAF GAF
 *  @brief Algorithm that provides device orientation. 
 *         Inputs are raw Accelerometer, Gyroscope and Magnetometer data. 
 *         Outputs are calibrated sensor and 9-axis sensor fusion. 
 *  @warning supported sampling frequency [50 Hz-1000 Hz]
 *  @warning supported gyroscope FSR [250 dps, 500 dps, 1000 dps, 2000 dps, 4000 dps]
 *  @warning supported accelerometer FSR [1 g, 2 g, 4 g, 8 g, 16 g, 32g]
 */

/** @struct invn_algo_gaf_struct
 *  Data structure holding internal algorithm state. 
 *  Ensure data buffer is aligned to 32 bits for 32-bits MCU.
 *  Other methods can be used to align memory using malloc or attribute((aligned, 4))
 *  @ingroup GAF
 */
typedef union invn_algo_gaf_struct {
	uint8_t data[INVN_ALGO_GAF_DATA_STRUCTURE_SIZE];
	uint32_t data32;
} invn_algo_gaf_struct;




/** @struct invn_algo_gaf_serif
 *  Serial interface structure 
 *  @ingroup GAF
 */
typedef struct {
	int (*read_reg)(uint8_t reg, uint8_t *buf, uint32_t len);
	int (*write_reg)(uint8_t reg, const uint8_t *buf, uint32_t len);
	void (*sleep_us)(uint32_t us);
} invn_algo_gaf_serif;

/** @struct invn_algo_gaf_input
 *  GAF input structure (raw data)
 *  @ingroup GAF
 */
typedef struct {
	int32_t sRacc_data[3];				/**< Raw accel in LSB - TDK sensor format */
	int32_t sRgyr_data[3];				/**< Raw gyro in LSB - TDK sensor format */
	int32_t sRmag_data[3];				/**< Raw mag in LSB - TDK sensor format */


	int32_t mask;						/**< Bit mask to specify updated inputs, INVN_ALGO_GAF_INPUT_MASK_ACC and/or INVN_ALGO_GAF_INPUT_MASK_GYR and/or INVN_ALGO_GAF_INPUT_MASK_MAG */
										/**< mask |=  INVN_ALGO_GAF_INPUT_MASK_ACC if new raw accel data is available */
										/**< mask |=  INVN_ALGO_GAF_INPUT_MASK_GYR if new raw gyro data is available */ 
										/**< mask |=  INVN_ALGO_GAF_INPUT_MASK_MAG if new raw mag data is available */
										/**< Warning - mask need to be reset by integrator after process function called  */
	
											
	/**< Optional inputs - set zero values if not available */
	/**< Best performances only when those inputs are available */
	int64_t sRimu_time_us;				/**< Timestamps of the input data in us */
	int16_t sRtemp_data;				/**< Raw temperature of the IMU sensor in LSB */

} invn_algo_gaf_input; 




	
/** @struct invn_algo_gaf_output
 *  GAF output structure (calibrated sensors and fusion output)
 *  @ingroup GAF
 */
typedef struct {
	/* Quaternion estimation, convention is WXYZ */
	int32_t grv_quat_q30[4];   /**< 6-axis (accel and gyro fusion) quaternion in q30 */
	int32_t rv_quat_q30[4];    /**< 9-axis (accel, gyro and mag fusion) quaternion in q30 */
	int32_t rv_accuracy_q27;   /**< 9-axis (accel, gyro and mag fusion) 3-sigma accuracy in rad in q27 format */
	int32_t gmrv_quat_q30[4];  /**< 6-axis (accel and mag fusion) quaternion in q30 */
	int32_t gmrv_accuracy_q27; /**< 9-axis (accel and mag fusion) 3-sigma accuracy in rad in q27 format*/
	int32_t gravity_q16[3];    /**< Gravity estimation in sensor frame (1 g = 1<<16) */
	int32_t linear_acc_q16[3]; /**< Linear acceleration estimation in sensor frame (1 g = 1<<16) */

	/* Calibration estimation */
	int32_t acc_uncal_q16[3]; /**< Uncalibrated accelerometer in q16 format (1 g = 1<<16) */
	int32_t acc_cal_q16[3];   /**< Calibrated accelerometer in q16 format (1 g = 1<<16) */
	int32_t acc_bias_q16[3];  /**< Accelerometer biases in q16 format (1 g = 1<<16)*/
	int32_t gyr_uncal_q16[3]; /**< Uncalibrated gyroscope in q16 format (1 dps = 1<<16) */
	int32_t gyr_cal_q16[3];   /**< Calibrated gyroscope in q16 format (1 dps = 1<<16) */
	int32_t gyr_bias_q16[3];  /**< Gyro biases in q16 format (1 dps = 1<<16)*/
	int32_t mag_uncal_q16[3]; /**< Uncalibrated magnetometer in q16 format (1uT = 1<<16) */
	int32_t mag_cal_q16[3];   /**< Calibrated magnetometer in q16 format (1uT = 1<<16) */
	int32_t mag_bias_q16[3];  /**< Magnetometer biases in q16 format (1uT = 1<<16) */

	/* Additional outputs */
	int32_t temp_degC_q16;           /**< Current temperature in q16 format (1 Cel = 1<<16) */
	int32_t gyr_HQ_bias_q16[3];      /**< High Quality Gyro biases in q16 format (1 dps = 1<<16) */
	int32_t gyr_HQ_bias_temperature; /**< Temperature in q16 format (1 Cel = 1<<16) corresponding to the HQ biases */
	int32_t gyr_MQ_bias_q16[3];      /**< Medium Quality Gyro biases in q16 format (1 dps = 1<<16) */
	
	int32_t mask;					/**< Bit mask to specify the updated output, INVN_ALGO_GAF_OUTPUT_MASK_ACCEL_CAL...  and INVN_ALGO_GAF_OUTPUT_MASK_DATAPROCESS */
											/**< if (mask & INVN_ALGO_GAF_OUTPUT_MASK_DATAPROCESS)==TRUE indicates that the input data was valid and has been processed */
											/**< if (mask & INVN_ALGO_GAF_INPUT_MASK_ACC)==TRUE a new calibrated accelerometer is available */
											/**< if (mask & INVN_ALGO_GAF_OUTPUT_MASK_QUAT_AG)==TRUE a new quaternion AG is available ... */


	/* Output calibration quality */
	int8_t acc_accuracy_flag;  /**< Accel accuracy from 0 (non calibrated) to 3 (well calibrated) */
	int8_t gyr_accuracy_flag;  /**< Gyro accuracy, from 0 (non calibrated) to 3 (well calibrated) */
	int8_t mag_accuracy_flag;  /**< Mag accuracy, from 0 (non calibrated) to 3 (well calibrated) */
	int8_t mag_anomalies_flag; /**< Mag anomalies detected since the last mag biases estimation was done:
                                    0 no mag anomaly, 1 medium mag anomaly, 2 high mag anomaly */
	int8_t stationary;         /**< Stationary state detection based on gyro data */
} invn_algo_gaf_output; 







/** @struct invn_algo_gaf_config
 *  GAF configuration structure (sensor related settings)
 *  @ingroup GAF
 */
typedef struct {
	/* Sensor generic configuration */
	
	/* Sensitivity */ 
	int32_t acc_fsr;                   /**< Accel full scale range (in g) (supported value:[1, 2, 4, 8, 16, 32]) */
	int32_t gyr_fsr;                   /**< Gyro full scale range (in dps) (supported value:[250, 500, 1000, 2000, 4000]) */
	int32_t mag_sc_q16;                /**< Mag sensitivity (in uT/LSB) (for example mag_uT = (mag_sc_q16 * raw_mag_lsb)/65536) */
	int32_t mag_staturation_limit_q16; /**< Mag saturated value in uT in q16 */
	int32_t temp_sensitivity;          /**< Temperature sensitivity in q30 (if temperature (Cel) = LSB * k + z, then temp_sensitivity = k) */
	int32_t temp_offset;               /**< Temperature offset in q16 (if temperature (Cel) = LSB * k + z, then temp_offset = z) */
	int32_t raw_data_format_flag;		/**< IMU format flag: value 0 or 1, 0 for normal resolution raw data, 1 for high resolution format
											0 => (Full Scale coded on 16 bit (FSR g = 1<<15 LSB)
											1 => (Full Scale coded on 20 bit (FSR g = 1<<19 LSB) */
	/* ODR configuration */
	uint32_t acc_odr_us;				/**< Raw Accelerometer input data rate in us */
	uint32_t gyr_odr_us;				/**< Raw Gyroscope input data rate in us */
	uint32_t mag_odr_us;				/**< Raw Magnetometer input data rate in us */
	int8_t   clock_variation;			/**< Clock variation, calculated as (actual clock - target clock) / target clock * (2^7-1) / 5 *100.
												Same format of the data in OTP reg SW_PLL1_TRIM. Default: 0. 
												If a valid sRimu_time_us is fill in the input structure, clock_variation will be ignored - Optional */ 

	int32_t  acc_pdr_us;				/**< Accelerometer internal processing data rate in us */
	int32_t  gyr_pdr_us;				/**< Gyroscope internal processing data rate in us */
	int32_t  mag_pdr_us;				/**< Magnetometer internal processing data rate in us */

	/* Biases configuration */  
	int32_t		acc_bias_q16[3];				/**< Last known accel biases. Set (0,0,0) is not available. Format is in q16 format (1 g = 1<<16)  */
	int32_t		gyr_bias_q16[3];				/**< Last known gyro biases. Set (0,0,0) is not available. Format is in q16 format (1 g = 1<<16)  */
	int32_t		mag_bias_q16[3];				/**< Last known mag biases. Set (0,0,0) is not available. Format is in q16 format (1 g = 1<<16)  */
	int8_t		acc_accuracy;				/**< Accuracy corresponding to the last known accel biases (0 to 3) */
	int8_t		gyr_accuracy;				/**< Accuracy corresponding to the last known gyro biases (0 to 3) */
	int32_t		gyr_bias_temperature;		/**< Temperature when the last gyro biases were estimated (in Cel in q16) */
	int8_t		mag_accuracy;				/**< Accuracy corresponding to the last known mag biases (0 to 3) */

	int8_t   enable_gyr_bias_flag;		/**< enable or disable the internal bias estimate, 
											when disable (0) the lastest estimated or loaded bias will used to calibrated the sensor
											when enable (1) the inline bias estimation is estimated when used to calibrated the sensor 
											when enable (2) the inline bias estimation with acc-assitance is estimated when used to calibrated the sensor */
	
	int8_t   enable_mag_bias_flag;		/**< enable or disable the internal bias estimate, 
											when disable (0) the lastest estimated or loaded bias will used to calibrated the sensor
											when enable (1) the inline bias estimation is estimated when used to calibrated the sensor 
											when enable (2) the inline bias estimation with gyro-assitance is estimated when used to calibrated the sensor */
	
	int8_t   enable_acc_bias_flag;		/**< enable or disable the internal bias estimate, 
											when disable (0) the lastest estimated or loaded bias will used to calibrated the sensor
											when enable (1) the inline bias estimation is estimated when used to calibrated the sensor 
											when enable (2) the inline bias estimation with gyro-assitance is estimated when used to calibrated the sensor */


	/* Stationary lock */
	int32_t stationary_angle_enable;            /**< Enable/disable stop fusion update when device is detected as static */
	int32_t stationary_angle_duration_us;       /**< Duration of stationary detection before stop fusion update (in us) */
	int32_t stationary_angle_threshold_deg_q16; /**< Threshold on cumulative angle before re-start to update the fusion in degree in q16 */

	/* Gyro calibration config */
	
	/* Parameters for MBE + FNM methods to allow new biases estimation */
	int32_t	strict_gyr_cal_threshold_metric1;     /**< Stationary detection threshold of 1st metric for the strict bias calibration */ 
	int32_t	strict_gyr_cal_threshold_metric2;     /**< Stationary detection threshold of 2nd metric for the strict bias calibration */
	int32_t loose_gyr_cal_stationary_duration_us; /**< Minimal duration detected as no motion before allow a gyroscope bias calibration */
	int32_t loose_gyr_cal_sample_num_log2;        /**< Gyro calibration number of samples (in log2) used to estimate metric1 and metric2 (also link to and minimum duration) */
	int32_t gyr_bias_reject_th;                   /**< Gyro bias rejection threshold (in q30 in 2000dps).
	                                                   Reverse formula is gyr_bias_reject_th*2000/2^30.
	                                                   Default is 3650722 corresponding to 6.8 dps. */
	
	/* Parameters for MBE only method to allow new bias estimation */
	int32_t loose_gyr_cal_threshold_metric1; /**< Stationary detection threshold of 1st metric for the loose bias calibration */
	int32_t loose_gyr_cal_threshold_metric2; /**< Stationary detection threshold of 2nd metric for the loose bias calibration */

	/* MBE Accel Protection parameters */
	int32_t acc_filtering_Npoints_log2;            /**< Accel filtering mean value, unit log 2 sample number, default value is 5 corresponding to a window for filtering of 32 samples */
	int32_t acc_square_sin_angle_motion_detect_th; /**< Square of sinus on angle threshold to reject gyro calibration
	                                                    Unit is (sin(theta)^2)*2^25.
	                                                    Default is 4318 corresponding to 0.65 degree. */
	
	/* MBE Bias validity parameters */
	uint32_t golden_bias_timer;                /**< Validity timer of the strict bias in sample number.
	                                                Default is 1440000 corresponding to 8 hours at 50Hz. */
	int32_t  golden_bias_temperature_validity; /**< Validity temperature variation of the strict bias (in degree q16).
	                                                Default value is 983040, corresponding to 15 degrees. */


	/* Magnetometer calibration RLS parameters */

	/* Magnetic disturbance rejection */
	int32_t  mag_thr_huge_disturbance; /**< Huge disturbance threshold to reset the algorithm - default 409600 // 200uT in q11 */
	int32_t  mag_thr_anomaly_radius;   /**< Difference between current radius and measurement to switch to disturbed state - default 30720 // 15uT in q11 */

	/* New point selection */
	int32_t  mag_thr_select_cos_angle;             /**< Minimum angular variation between 2 consecutive measurements (using gyroscope) - default 1050277989 // cos(12°), q30 */
	int32_t  mag_thr_select_min_distance_assisted; /**< Minimum distance variation between 2 consecutive measurements (using gyroscope) - default is 10240 //  5uT, q11 */
	int32_t  mag_thr_select_distance;              /**< Minimum distance variation between 2 consecutive measurements (not using gyroscope) - default 30720 // 15uT, q11 */

	/* RLS filter parameters */
	int32_t  mag_rls_Q0_standalone; /**< Model covariance - default 3 */
	int32_t  mag_rls_R0_standalone; /**< Measurement covariance - default 400000 */
	int32_t  mag_rls_Q0_assisted;   /**< Model covariance for gyroscope assisted model - default is 1 */
	int32_t  mag_rls_R0_assisted;   /**< Measurement covariance for gyroscope assisted model - default is 5000 */

	/* Post-treatment parameters */
	int32_t  mag_thr_max_radius_jump;    /**< Maximum estimated magnetic radius jump between 2 solutions - default  40960 // 20uT in q11 */
	int32_t  mag_thr_max_radius;         /**< Maximum field radius - default 256000 // 125uT, q11; */
	int32_t  mag_thr_min_radius;         /**< Minimum field radius - default 36864 // 18uT, q11 */
	int32_t  mag_thr_cov_accuracy_lvl1_; /**< Covariance corresponding to accuracy 1, default value is 10000 */
	int32_t  mag_thr_cov_accuracy_lvl2_; /**< Covariance corresponding to accuracy 2, default value is 5000 */
	int32_t  mag_thr_cov_accuracy_lvl3_; /**< Covariance corresponding to accuracy 3, default value is 2500 */

	// Parameters to avoid getting stuck in RLS filter after a previous solution is accepted (e.g. Mag-Safe case)
	int32_t cov_stuck_convergence_thr;	/*!< parameter to determine if current covariance solution could stuck covergence when RLS is internally reset and target covariance is far */
	int32_t cov_fast_convergence_thr;	/*!< parameter to set new covariance target when possible stuck covariance is detected */

	// Parameters to confront Mag Bias solutions on Norm Metric
	int32_t norm_p2p_cand_thr; /*!< Threshold of Norm P2P value above which candidate solution is rejected. Format is uT in Q11. */
	int32_t norm_p2p_solu_thr; /*!< Threshold of Norm P2P value below which previous RLS solution is retained. Format is uT in Q11. */

	// Parameter to control if we execute "Flag4" condition on covariance to reach 
	int16_t enable_flag4;

	// Parameter to indicate buffer size of latest RLS processed samples
	uint8_t uncal_mag_buffer_size; /*!< Size of Uncal Mag data buffer used to benchmark bias candidates. Greatest size supported is 10 samples (default is 8). */
								   
	/* Accel calibration RLS parameters */
	 /* New point selection */
	int32_t acc_thr_select_cos_angle;				/**< Minimum angular variation between 2 consecutive measurements (using gyroscope) - default 1050277989 // cos(12 degree), q30 */
	int32_t acc_thr_select_min_distance_assisted;	/**< Minimum distance variation between 2 consecutive measurements (using gyroscope) - default is 6554 //  100mg */
	int32_t acc_thr_select_distance;				/**< Minimum distance variation between 2 consecutive measurements (not using gyroscope) - default 26214 // 400mg */

	/* RLS filter parameters */
	int32_t acc_rls_Q0_standalone;					/**< Model covariance - default 3 */
	int32_t acc_rls_R0_standalone;					/**< Measurement covariance - default 32768 */
	int32_t acc_rls_Q0_assisted;					/**< Model covariance for gyroscope assisted model - default is 1 */
	int32_t acc_rls_R0_assisted;					/**< Measurement covariance for gyroscope assisted model - default is 32768 */

	/* Post-treatment parameters */
	int32_t acc_thr_cov_accuracy_lvl1;				/**< Covariance corresponding to accuracy 1, default value is 10000 */
	int32_t acc_thr_cov_accuracy_lvl2;				/**< Covariance corresponding to accuracy 2, default value is 5000 */
	int32_t acc_thr_cov_accuracy_lvl3;				/**< Covariance corresponding to accuracy 3, default value is 2500 */

	/* Orientation */ 
	int32_t	fus_high_speed_drift;           /**< Percentage of error on gyroscope integration (1% = 32767).
	                                             This error covers gyroscope sensitivity, timestamp, and quantization */
	int32_t	fus_low_speed_drift_roll_pitch; /**< Gyroscope integration error related to bias precision.
	                                             Higher value increase accel roll/pitch correction in steady state*/
	int32_t	fus_low_speed_drift_yaw;        /**< Gyroscope integration error related to bias precision.
	                                             Higher value increase compass yaw correction in steady state */
	int32_t	fus_measurement_covariance_acc; /**< Accelerometer measurement covariance in q15 (1G^2 = 32767) */
	int32_t	fus_measurement_covariance_mag; /**< Magnetometer measurement covariance in q15 (1uT^2 = 32767) */
	int32_t	fus_mag_anomaly_rejection;      /**< Magnetic anomaly rejection (Max rejection is 100% = 1073741824L, No rejection is 0% = 0L). */
	int32_t	fus_acceleration_rejection;     /**< Linear acceleration rejection (Max rejection is 100% = 1073741824L, No rejection is 0% = 0L). */
	int32_t fus_thresholdLockA;             /**< Threshold to detect steady accelerometer, e.g. 30mg --> value in q30 square (0.03g*2^25)^2/2^30. */
	int32_t fus_thresholdLockM;             /**< Threshold to detect steady magnetometer, e.g. 1.5uT = 15mG --> value in q30 square (15mG*2^16)^2/2^30. */
	int32_t fus_lockTime_ms;                /**< Time (ms) to get into lock mode once accel and mag are detected as steady. */

	// Delta Bias trick
	int32_t mag_delta_bias_q16; // delta bias functionality to reset the fusion internal quaternion, default value is 655360, i.e. 10µT in q16

	// stop yaw convergence
	int32_t thresh_yaw_stop_convergence_q30; /*!< parameter to stop yaw error correction when value to correct is lower than threshold. Unit is radians in q30. Default value is -1 (desactivated feature). */
	int32_t thresh_yaw_smooth_convergence_q15;	/*!< parameter to control yaw error correction speed as a percentage of the gyro speed. Unit is percentage in q15 (e.g., for 10%, set value to 3277). Default value is -1 (desactivated feature). */


} invn_algo_gaf_config;

typedef struct {

	/* ODR configuration for on-line update */
	int32_t acc_odr_us;					/**< Raw Accelerometer input data rate in us */
	int32_t gyr_odr_us;					/**< Raw Gyroscope input data rate in us */
	int32_t mag_odr_us;					/**< Raw Magnetometer input data rate in us */

	int32_t acc_pdr_us;					/**< Accelerometer internal processing data rate in us */
	int32_t gyr_pdr_us;					/**< Gyroscope internal processing data rate in us */
	int32_t mag_pdr_us;					/**< Magnetometer internal processing data rate in us */


} invn_algo_gaf_odr_config;


/** @brief Returns library version x.y.z-suffix as a char array.
 *  @return  Version x.y.z-suffix as char array.
 *  @ingroup GAF
 */
const char *invn_algo_gaf_version(void);

/** @brief Generate default configuration.
 *  Requires to provide sensors ODR to tune parameters accordingly.
 *  @param[out] config            Pointer to algorithm configuration structure.
 *  @param[in]  sensor_odr_us[3]  Sensors ODR in us in the following order: Accel, Gyro and Mag.
 *  @return                       0 on success, negative value on error.
 *  @ingroup GAF
 */
int invn_algo_gaf_generate_config(invn_algo_gaf_config *config, int32_t sensor_odr_us[3]);

/** @brief Initializes algorithm.
 *  @param[in] self    Pointer to algorithm object.
 *  @param[in] serif   Serial interface object for communication with IMU.
 *  @param[in] config  Pointer to algorithm configuration structure.
 *  @return            0 on success, negative value on error.
 *  @ingroup GAF
 */
int invn_algo_gaf_init(invn_algo_gaf_struct *self, const invn_algo_gaf_serif *serif,
                       const invn_algo_gaf_config *config);

/** @brief Sets algorithm configuration.
 *  @param[in] self    Pointer to algorithm object.
 *  @param[in] config  Pointer to algorithm configuration.
 *  @return            0 on success, negative value on error.
 *  @ingroup GAF
 */
int invn_algo_gaf_set_config(invn_algo_gaf_struct *self, const invn_algo_gaf_config *config);

/** @brief Get current algorithm configuration - for save/reload purposes.
 *  @param[in] self    Pointer to algorithm object.
 *  @param[in] config  Pointer to algorithm configuration.
 *  @return            0 on success, negative value on error.
 *  @ingroup GAF
 */
int invn_algo_gaf_get_config(invn_algo_gaf_struct *self, invn_algo_gaf_config *config);


/** @brief Get current algorithm sensor bias used by the algorithm.
 *  @param[in] self    Pointer to algorithm object.
 *  @param[out] acc_bias[3]  Pointer to acc bias.
 *  @param[out] acc_accuracy  Pointer to acc accuracy.
 *  @param[out] gyr_bias[3]  Pointer to gyr bias.
 *  @param[out] gyr_accuracy  Pointer to gyr accuracy.
 *  @param[out] mag_bias[3]  Pointer to mag bias.
 *  @param[out] mag_accuracy  Pointer to mag accuracy.
 *  @return            0 on success, negative value on error.
 *  @ingroup GAF
 */
int invn_algo_gaf_get_bias(invn_algo_gaf_struct *self, int32_t acc_bias[3], int8_t* acc_accuracy, int32_t gyr_bias[3], int8_t* gyr_accuracy, int32_t mag_bias[3], int8_t* mag_accuracy);

/** @brief Sets algorithm ODR on fly.
 *  @param[in] self    Pointer to algorithm object.
 *  @param[in] odr_config  Pointer to algorithm ODR configuration.
 *  @return            0 on success, negative value on error.
 *  @ingroup AccCal
 */
int invn_algo_gaf_set_odr(invn_algo_gaf_struct *self, const invn_algo_gaf_odr_config *odr_config);

/** @brief Enable or disable the inline bias estimation.
 *  @param[in] self    Pointer to algorithm object.
 *  @param[in] enable_bias_flag  per sensor ( order Accel, Gyro , Mag) disable: 0, enable: 1, enable gyro-assisted/acc-assisted: 2
 *  @return            0 on success, negative value on error.
 *  @ingroup SensorCal
 */
int invn_algo_gaf_start_stop_bias_estimate(invn_algo_gaf_struct *self, const int32_t enable_sensor_bias_flag[3]);


/** @brief reset calibration Algorithm.
 *  call reset calibration and keep all configurations
 *  @param[in] self    Pointer to algorithm object.
 *  @param[in] bias_reload_flag[3] tab of bias status per sensor (order Accel, Gyro, Mag), value 0: reset bias, value 1: current bias and accuracy are maintained 
 *  @return            0 on success, negative value on error.
 *  @ingroup GAF
 */
int invn_algo_gaf_reset_calibration(invn_algo_gaf_struct *self, const int32_t bias_reload_flag[3]);

/** @brief Reload Magnetometer configuration and espectially bias and reset Mag calibration
 *  @param[in] self    Pointer to algorithm object -  call reset function and keep all configurations
 *  @param[in] config  Pointer to algorithm configuration. *  @return            0 on success, negative value on error.
 *  @ingroup GAF
 */
int invn_algo_gaf_reload_mag(invn_algo_gaf_struct *self, const  invn_algo_gaf_config *config);

/** @brief Processes data.
 *  @param[in]  self     Pointer to algorithm object.
 *  @param[in]  inputs   Algorithm input data.
 *  @param[out] outputs  Algorithm output data.
 *  @return              0 on success, negative value on error.
 *  @ingroup GAF
 */
int invn_algo_gaf_process(invn_algo_gaf_struct *self, const invn_algo_gaf_input *inputs,
                          invn_algo_gaf_output *outputs);




#ifdef __cplusplus
}
#endif

#endif
