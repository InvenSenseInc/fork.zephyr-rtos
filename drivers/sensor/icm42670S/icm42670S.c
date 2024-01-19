/*
 * Copyright (c) 2023 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor/icm42670s.h>

#include "icm42670S.h"

LOG_MODULE_REGISTER(ICM42670S, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "ICM42670S driver enabled without any devices"
#endif



static inline int icm42670S_bus_check(const struct device *dev)
{
	const struct icm42670S_config *cfg = dev->config;

	return cfg->bus_io->check(&cfg->bus);
}

static inline int icm42670S_reg_read(const struct device *dev,
				  uint8_t reg, uint8_t *buf, uint32_t size)
{
	const struct icm42670S_config *cfg = dev->config;

	return cfg->bus_io->read(&cfg->bus, reg, buf, size);
}

static inline int inv_io_hal_read_reg(struct inv_imu_serif *serif, 
				uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
	return icm42670S_reg_read(serif->context, reg, rbuffer, rlen);
}

static inline int icm42670S_reg_write(const struct device *dev,
				   uint8_t reg, const uint8_t *buf, uint32_t size)
{
	const struct icm42670S_config *cfg = dev->config;

	return cfg->bus_io->write(&cfg->bus, reg, buf, size);
}

static inline int inv_io_hal_write_reg(struct inv_imu_serif *serif, uint8_t reg, 
                         const uint8_t *wbuffer, uint32_t wlen)
{
	return icm42670S_reg_write(serif->context, reg, wbuffer, wlen);
}

void inv_imu_sleep_us(uint32_t us)
{
	k_sleep(K_USEC(us));
}

uint64_t inv_imu_get_time_us(void)
{
	// returns the elapsed time since the system booted, in milliseconds
	return k_uptime_get() * 1000;
}

static int icm42670S_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct icm42670S_data *data = dev->data;
	int      status = 0;
	uint8_t  int_status;
	uint16_t total_packet_count = 0;
	uint16_t packet_size        = FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE +
	                       FIFO_TEMP_DATA_SIZE + FIFO_TS_FSYNC_SIZE;
	
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	/* Ensure data ready status bit is set */
	if (status |= inv_imu_read_reg(&data->driver, INT_STATUS, 1, &int_status))
		return status;

	if ((int_status & INT_STATUS_FIFO_THS_INT_MASK) ||
	    (int_status & INT_STATUS_FIFO_FULL_INT_MASK)) {
		uint8_t  f_data[2];
		uint16_t packet_count;

/*
		 * Make sure RCOSC is enabled to guarrantee FIFO read */
		status |= inv_imu_switch_on_mclk(&data->driver);
		
		/* FIFO record mode configured at driver init, so we read packet number, not byte count */
		if ((status |= inv_imu_read_reg(&data->driver, FIFO_COUNTH, 2, &f_data[0])) != INV_ERROR_SUCCESS) {
			return status;
		}

		total_packet_count = (uint16_t)(f_data[0] | (f_data[1] << 8));
		packet_count       = total_packet_count;
		while (packet_count > 0) {
			uint16_t invalid_frame_cnt = 0;
			/* Read FIFO only when data is expected in FIFO */
			/* fifo_idx type variable must be large enough to parse the FIFO_MIRRORING_SIZE */
			uint16_t fifo_idx = 0;

			if (status |=
			    inv_imu_read_reg(&data->driver, FIFO_DATA, packet_size * packet_count, &data->driver.fifo_data)) {
				/* 
				 * Sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
				 * reset FIFO and try next chance 
				 */
				status |= inv_imu_reset_fifo(&data->driver);
				status |= inv_imu_switch_off_mclk(&data->driver);
				return status;
			}

			for (uint16_t i = 0; i < packet_count; i++) {
				const fifo_header_t *  header;

				header            = (fifo_header_t *)&data->driver.fifo_data[fifo_idx];
				fifo_idx += FIFO_HEADER_SIZE;

				/* Decode invalid frame */
				if (header->Byte == 0x80) {
					uint8_t is_invalid_frame = 1;
					/* Check N-FIFO_HEADER_SIZE remaining bytes are all 0 to be invalid frame */
					for (uint8_t j = 0; j < (packet_size - FIFO_HEADER_SIZE); j++) {
						if (data->driver.fifo_data[fifo_idx + j]) {
							is_invalid_frame = 0;
							break;
						}
					}
					/* In case of invalid frame read FIFO will be retried for this packet */
					invalid_frame_cnt += is_invalid_frame;
					fifo_idx += packet_size - FIFO_HEADER_SIZE;
				} else {
					/* Decode packet */
					if (header->bits.msg_bit) {
						/* MSG BIT set in FIFO header, Resetting FIFO */
						status |= inv_imu_reset_fifo(&data->driver);
						status |= inv_imu_switch_off_mclk(&data->driver);
						return status < 0 ? status : INV_ERROR;
					}

					if (header->bits.accel_bit) {
						data->accel[0] = (int16_t)((data->driver.fifo_data[0 + fifo_idx] << 8) | data->driver.fifo_data[1 + fifo_idx]);
						data->accel[1] = (int16_t)((data->driver.fifo_data[2 + fifo_idx] << 8) | data->driver.fifo_data[3 + fifo_idx]);
						data->accel[2] = (int16_t)((data->driver.fifo_data[4 + fifo_idx] << 8) | data->driver.fifo_data[5 + fifo_idx]);
						fifo_idx += FIFO_ACCEL_DATA_SIZE;
					}

					if (header->bits.gyro_bit) {
						data->gyro[0] = (int16_t)((data->driver.fifo_data[0 + fifo_idx] << 8) | data->driver.fifo_data[1 + fifo_idx]);
						data->gyro[1] = (int16_t)((data->driver.fifo_data[2 + fifo_idx] << 8) | data->driver.fifo_data[3 + fifo_idx]);
						data->gyro[2] = (int16_t)((data->driver.fifo_data[4 + fifo_idx] << 8) | data->driver.fifo_data[5 + fifo_idx]);
						fifo_idx += FIFO_GYRO_DATA_SIZE;
					}

					/* 
					 * The coarse temperature (8B FIFO packet format) 
					 * range is ± 64 degrees with 0.5°C resolution.
					 */
					/* cast to int8_t since FIFO is in 16 bits mode (temperature on 8 bits) */
					data->temperature = (int8_t)data->driver.fifo_data[0 + fifo_idx];
					fifo_idx += FIFO_TEMP_DATA_SIZE;

				} /* end of else invalid frame */
			} /* end of FIFO read for loop */
			packet_count = invalid_frame_cnt;
		} /*end of while: packet_count > 0*/

		status |= inv_imu_switch_off_mclk(&data->driver);
		if (status < 0)
			return status;

	} /*else: FIFO threshold was not reached and FIFO was not full*/

	return 0;
}

static void icm42670S_accel_convert(struct sensor_value *val, int raw_val, uint16_t fs)
{
	int64_t conv_val;

	/* 16 bit accelerometer. 2^15 bits represent the range in G */
	conv_val = (int64_t)raw_val * SENSOR_G * fs / INT16_MAX;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;

}

static void icm42670S_gyro_convert(struct sensor_value *val, int16_t raw_val, uint16_t fs)
{
	int64_t conv_val;

	/* 16 bit gyroscope. 2^15 bits represent the range in degrees/s */
	conv_val = ((int64_t)raw_val * fs * SENSOR_PI) / (INT16_MAX * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

static void icm42670S_temp_convert(struct sensor_value *val, int16_t raw_val)
{
	int64_t conv_val;

	conv_val = 25 * 1000000 + ((int64_t)raw_val * 1000000 / 2);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

static int icm42670S_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct icm42670S_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm42670S_accel_convert(val, data->accel[0], data->accel_fs);
		icm42670S_accel_convert(val + 1, data->accel[1], data->accel_fs);
		icm42670S_accel_convert(val + 2, data->accel[2], data->accel_fs);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm42670S_gyro_convert(val, data->gyro[0], data->gyro_fs);
		icm42670S_gyro_convert(val + 1, data->gyro[1], data->gyro_fs);
		icm42670S_gyro_convert(val + 2, data->gyro[2], data->gyro_fs);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm42670S_temp_convert(val, data->temperature);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static uint32_t convert_freq_to_bitfield(uint32_t val)
{
	uint32_t odr_bitfield = 0;
	
	if (val < 3 && val >= 1) {
		odr_bitfield = ACCEL_CONFIG0_ODR_1_5625_HZ; /*(= GYRO_CONFIG0_ODR_1_5625_HZ )*/
	} else if (val < 6 && val >= 3) {
		odr_bitfield = ACCEL_CONFIG0_ODR_3_125_HZ; /*(= GYRO_CONFIG0_ODR_3_125_HZ )*/
	} else if (val < 12 && val >= 6) {
		odr_bitfield = ACCEL_CONFIG0_ODR_6_25_HZ; /*(= GYRO_CONFIG0_ODR_6_25_HZ )*/
	} else if (val < 25 && val >= 12) {
		odr_bitfield = ACCEL_CONFIG0_ODR_12_5_HZ; /*(= GYRO_CONFIG0_ODR_12_5_HZ )*/
	} else if (val < 50 && val >= 25) {
		odr_bitfield = ACCEL_CONFIG0_ODR_25_HZ; /*(= GYRO_CONFIG0_ODR_25_HZ )*/
	} else if (val < 100 && val >= 50) {
		odr_bitfield = ACCEL_CONFIG0_ODR_50_HZ; /*(GYRO_CONFIG0_ODR_50_HZ)*/
	} else if (val < 200 && val >= 100) {
		odr_bitfield = ACCEL_CONFIG0_ODR_100_HZ; /*(= GYRO_CONFIG0_ODR_100_HZ )*/
	} else if (val < 400 && val >= 200) {
		odr_bitfield = ACCEL_CONFIG0_ODR_200_HZ; /*(= GYRO_CONFIG0_ODR_200_HZ )*/
	} else if (val < 800 && val >= 400) {
		odr_bitfield = ACCEL_CONFIG0_ODR_400_HZ; /*(= GYRO_CONFIG0_ODR_400_HZ )*/
	} else if (val < 1600 && val >= 800) {
		odr_bitfield = ACCEL_CONFIG0_ODR_800_HZ; /*(= GYRO_CONFIG0_ODR_800_HZ )*/
	} else if (val == 1600 ) {
		odr_bitfield = ACCEL_CONFIG0_ODR_1600_HZ; /*(= GYRO_CONFIG0_ODR_1600_HZ )*/
	}
	return odr_bitfield;
}

static uint32_t convert_acc_fs_to_bitfield(uint32_t val, uint8_t *fs)
{
	uint32_t odr_bitfield = 0;
	
	if (val < 4 && val >= 2) {
		odr_bitfield = ACCEL_CONFIG0_FS_SEL_2g;
		*fs = 2;
	} else if (val < 8 && val >= 4) {
		odr_bitfield = ACCEL_CONFIG0_FS_SEL_4g;
		*fs = 4;
	} else if (val < 16 && val >= 8) {
		odr_bitfield = ACCEL_CONFIG0_FS_SEL_8g;
		*fs = 8;
	} else if (val == 16) {
		odr_bitfield = ACCEL_CONFIG0_FS_SEL_16g;
		*fs = 16;
	}
	return odr_bitfield;
}

static uint32_t convert_gyr_fs_to_bitfield(uint32_t val, uint16_t *fs)
{
	uint32_t odr_bitfield = 0;
	
	if (val < 500 && val >= 250) {
		odr_bitfield = GYRO_CONFIG0_FS_SEL_250dps;
		*fs = 250;
	} else if (val < 1000 && val >= 500) {
		odr_bitfield = GYRO_CONFIG0_FS_SEL_500dps;
		*fs  = 500;
	} else if (val < 2000 && val >= 1000) {
		odr_bitfield = GYRO_CONFIG0_FS_SEL_1000dps;
		*fs = 1000;
	} else if (val == 2000) {
		odr_bitfield = GYRO_CONFIG0_FS_SEL_2000dps;
		*fs = 2000;
	}
	return odr_bitfield;
}

static uint32_t convert_ln_bw_to_bitfield(uint32_t val)
{
	uint32_t odr_bitfield = 0xFF;
	
	if (val < 25 && val >= 16) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_BW_16; /* (= GYRO_CONFIG1_GYRO_FILT_BW_16) */
	} else if (val < 34 && val >= 25) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_BW_25; /* (= GYRO_CONFIG1_GYRO_FILT_BW_25) */
	} else if (val < 53 && val >= 34) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_BW_34; /* (= GYRO_CONFIG1_GYRO_FILT_BW_34) */
	} else if (val < 73 && val >= 53) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_BW_53; /* (= GYRO_CONFIG1_GYRO_FILT_BW_53) */
	} else if (val < 121 && val >= 73) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_BW_73; /* (= GYRO_CONFIG1_GYRO_FILT_BW_73) */
	} else if (val < 180 && val >= 121) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_BW_121; /* (= GYRO_CONFIG1_GYRO_FILT_BW_121) */
	} else if (val == 180) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_BW_180; /* (= GYRO_CONFIG1_GYRO_FILT_BW_180) */
	} else if (val == 0) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_BW_NO_FILTER; /* (= GYRO_CONFIG1_GYRO_FILT_BW_NO_FILTER) */
	}
	return odr_bitfield;
}

static uint32_t convert_lp_avg_to_bitfield(uint32_t val)
{
	uint32_t odr_bitfield = 0xFF;
	
	if (val < 4 && val >= 2) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_AVG_2;
	} else if (val < 8 && val >= 4) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_AVG_4; 
	} else if (val < 16 && val >= 8) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_AVG_8; 
	} else if (val < 32 && val >= 16) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_AVG_16; 
	} else if (val < 64 && val >= 32) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_AVG_32; 
	} else if (val == 64) {
		odr_bitfield = ACCEL_CONFIG1_ACCEL_FILT_AVG_64;
	}
	return odr_bitfield;
}

static int icm42670S_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	struct icm42670S_data *drv_data = dev->data;
	int err = 0;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_CONFIGURATION) {
			if (val->val1 == ICM42670S_POWER_OFF) {
				err |= inv_imu_disable_accel(&drv_data->driver);
			} else if (val->val1 == ICM42670S_LOW_POWER_MODE) {
				err |= inv_imu_enable_accel_low_power_mode(&drv_data->driver);
			} else if (val->val1 == ICM42670S_LOW_NOISE_MODE) {
				err |= inv_imu_enable_accel_low_noise_mode(&drv_data->driver);
			} else {
				LOG_ERR("Not supported ATTR value");
				return -EINVAL;
			}
				
		} else if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			if (val->val1 > 1600 || val->val1 < 1) {
				LOG_ERR("Incorrect sampling value");
				return -EINVAL;
			} else {
				err |= inv_imu_set_accel_frequency(&drv_data->driver, 
						convert_freq_to_bitfield(val->val1));	
			}
		
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			if (val->val1 > 16 || val->val1 < 2) {
				LOG_ERR("Incorrect fullscale value");
				return -EINVAL;
			} else {
				err |= inv_imu_set_accel_fsr(&drv_data->driver, 	
						convert_acc_fs_to_bitfield(val->val1, &drv_data->accel_fs));
				LOG_DBG("Set accel full scale to: %d G", drv_data->accel_fs);
			}
			
		} else if ((enum sensor_attribute_icm42670S)attr == SENSOR_ATTR_BW_FILTER_LPF) {
			if (val->val1 > 180) {
				LOG_ERR("Incorrect low pass filter bandwith value");
				return -EINVAL;
			} else {
				err |= inv_imu_set_accel_ln_bw(&drv_data->driver,
						convert_ln_bw_to_bitfield(val->val1));
			}
			
		} else if ((enum sensor_attribute_icm42670S)attr == SENSOR_ATTR_AVERAGING) {
			if (val->val1 > 64 || val->val1 < 2) {
				LOG_ERR("Incorrect averaging filter value");
				return -EINVAL;
			} else {
				err |= inv_imu_set_accel_lp_avg(&drv_data->driver,
						convert_lp_avg_to_bitfield(val->val1));
			}
			
		}else {
			LOG_ERR("Not supported ATTR");
			return -ENOTSUP;
		}
		break;
		
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_CONFIGURATION) {
			if (val->val1 == ICM42670S_POWER_OFF) {
				err |= inv_imu_disable_gyro(&drv_data->driver);
			} else if (val->val1 == ICM42670S_LOW_NOISE_MODE) {
				err |= inv_imu_enable_gyro_low_noise_mode(&drv_data->driver);
			} else {
				LOG_ERR("Not supported ATTR value");
				return -EINVAL;
			}
			
		} else if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			if (val->val1 > 8000 || val->val1 < 12) {
				LOG_ERR("Incorrect sampling value");
				return -EINVAL;
			} else {
				err |= inv_imu_set_gyro_frequency(&drv_data->driver, 
						convert_freq_to_bitfield(val->val1));
			}
		
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			if (val->val1 > 2000 || val->val1 < 250) {
				LOG_ERR("Incorrect fullscale value");
				return -EINVAL;
			} else {
				err |= inv_imu_set_gyro_fsr(&drv_data->driver, 	
						convert_gyr_fs_to_bitfield(val->val1, &drv_data->gyro_fs));
				LOG_DBG("Set gyro fullscale to: %d dps", drv_data->gyro_fs);
			}
			
		} else if ((enum sensor_attribute_icm42670S)attr == SENSOR_ATTR_BW_FILTER_LPF) {
			if (val->val1 > 180) {
				LOG_ERR("Incorrect low pass filter bandwith value");
				return -EINVAL;
			} else {
				err |= inv_imu_set_gyro_ln_bw(&drv_data->driver,
						convert_ln_bw_to_bitfield(val->val1));
			}
			
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}
		break;
	
	default:
		LOG_ERR("Not supported");
		return -EINVAL;
	}

	return err;
}

static const struct sensor_driver_api icm42670S_api_funcs = {
	.sample_fetch = icm42670S_sample_fetch,
	.channel_get = icm42670S_channel_get,
	.trigger_set = icm42670S_trigger_set,
	.attr_set = icm42670S_attr_set,
};

static int icm42670S_chip_init(const struct device *dev)
{
	struct icm42670S_data *data = dev->data;
	
	int err = icm42670S_bus_check(dev);
	if (err < 0) {
		LOG_DBG("bus check failed: %d", err);
		return err;
	}
	k_sleep(K_SECONDS(0.3));

	// Initialize serial interface and device
	data->serif.context = dev;
	data->serif.read_reg = inv_io_hal_read_reg;
	data->serif.write_reg = inv_io_hal_write_reg;
	data->serif.max_read = 1024 * 32;
	data->serif.max_write = 1024 * 32;
	data->serif.serif_type = UI_I2C;
	err = inv_imu_init(&data->driver, &data->serif, NULL);
	if (err < 0) {
		LOG_DBG("Init failed: %d", err);
		return err;
	}
	
	err = inv_imu_get_who_am_i(&data->driver, &data->chip_id);
	if (err < 0) {
		LOG_DBG("ID read failed: %d", err);
		return err;
	}

	if (data->chip_id == INV_IMU_WHOAMI) {
		LOG_DBG("ID OK");
	} else {
		LOG_DBG("bad chip id 0x%x", data->chip_id);
		return -ENOTSUP;
	}
	
	err = icm42670S_init_interrupt(dev);
	if (err < 0) {
		LOG_ERR("Failed to initialize interrupt.");
		return err;
	}
	
	// Default fullscale options
	data->accel_fs = 16;
	data->gyro_fs = 2000;
	
	k_sleep(K_MSEC(1));

	LOG_DBG("\"%s\" OK", dev->name);
	return 0;
}

/* Initializes a struct icm42670S_config for an instance on a SPI bus. */
#define ICM42670S_CONFIG_SPI(inst)				\
	{						\
		.bus.spi = SPI_DT_SPEC_INST_GET(	\
			inst, ICM42670S_SPI_OPERATION, 0),	\
		.bus_io = &icm42670S_bus_io_spi,		\
		.gpio_int = GPIO_DT_SPEC_INST_GET(inst, int_gpios),    \
	}

/* Initializes a struct icm42670S_config for an instance on an I2C bus. */
#define ICM42670S_CONFIG_I2C(inst)			       \
	{					       \
		.bus.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.bus_io = &icm42670S_bus_io_i2c,	       \
		.gpio_int = GPIO_DT_SPEC_INST_GET(inst, int_gpios),    \
	}

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define ICM42670S_DEFINE(inst)						\
	static struct icm42670S_data icm42670S_data_##inst;			\
	static const struct icm42670S_config icm42670S_config_##inst =	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			    (ICM42670S_CONFIG_SPI(inst)),			\
			    (ICM42670S_CONFIG_I2C(inst)));			\
									\
	PM_DEVICE_DT_INST_DEFINE(inst, icm42670S_pm_action);		\
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			 icm42670S_chip_init,				\
			 PM_DEVICE_DT_INST_GET(inst),			\
			 &icm42670S_data_##inst,				\
			 &icm42670S_config_##inst,				\
			 POST_KERNEL,					\
			 CONFIG_SENSOR_INIT_PRIORITY,			\
			 &icm42670S_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(ICM42670S_DEFINE)
