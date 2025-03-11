/*
 * Copyright (c) 2024 TDK Invensense
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2020 TDK Invensense
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
#include <zephyr/drivers/sensor/icm456xx.h>
#include <zephyr/drivers/sensor/tdk_apex.h>

#include "icm456xx.h"
#include "icm456xx_trigger.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ICM456XX, CONFIG_SENSOR_LOG_LEVEL);

/* Convert DT enum to sensor ODR selection */
#define ICM456XX_CONVERT_ENUM_TO_ODR_POS (4)

static inline int icm456xx_reg_read(const struct device *dev, uint8_t reg, uint8_t *buf,
		uint32_t size)
{
	const struct icm456xx_config *cfg = dev->config;
	return cfg->bus_io->read(&cfg->bus, reg, buf, size);
}

static inline int inv_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer,
		uint32_t rlen)
{
	return icm456xx_reg_read(context, reg, rbuffer, rlen);
}

static inline int icm456xx_reg_write(const struct device *dev, uint8_t reg, const uint8_t *buf,
		uint32_t size)
{
	const struct icm456xx_config *cfg = dev->config;

	return cfg->bus_io->write(&cfg->bus, reg, (uint8_t *)buf, size);
}

static inline int inv_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer,
		uint32_t wlen)
{
	return icm456xx_reg_write(context, reg, wbuffer, wlen);
}


void inv_sleep_us(uint32_t us)
{
	k_sleep(K_USEC(us));
}

uint64_t inv_imu_get_time_us(void)
{
	/* returns the elapsed time since the system booted, in milliseconds */
	return k_uptime_get() * 1000;
}

static uint16_t convert_dt_enum_to_freq(uint8_t val)
{
	uint16_t freq;

	switch (val) {
	case 0:
		freq = 0;
		break;
	case 1:
		freq = 6400;
		break;
	case 2:
		freq = 3200;
		break;
	case 3:
		freq = 1600;
		break;
	case 4:
		freq = 800;
		break;
	case 5:
		freq = 400;
		break;
	case 6:
		freq = 200;
		break;
	case 7:
		freq = 100;
		break;
	case 8:
		freq = 50;
		break;
	case 9:
		freq = 25;
		break;
	case 10:
		freq = 12;
		break;
	case 11:
		freq = 6;
		break;
	case 12:
		freq = 3;
		break;
	case 13:
		freq = 1;
		break;
	default:
		freq = 0;
		break;
	}
	return freq;
}

uint32_t convert_freq_to_enum(uint32_t val, uint16_t *freq)
{
        uint32_t odr_enum = 0;

        if (val < 3 && val >= 1) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ; /*(= GYRO_CONFIG0_ODR_1_5625_HZ )*/
                *freq = 1;
        } else if (val < 6 && val >= 3) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_3_125_HZ; /*(= GYRO_CONFIG0_ODR_3_125_HZ )*/
                *freq = 3;
        } else if (val < 12 && val >= 6) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ; /*(= GYRO_CONFIG0_ODR_6_25_HZ )*/
                *freq = 6;
        } else if (val < 25 && val >= 12) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ; /*(= GYRO_CONFIG0_ODR_12_5_HZ )*/
                *freq = 12;
        } else if (val < 50 && val >= 25) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_25_HZ; /*(= GYRO_CONFIG0_ODR_25_HZ )*/
                *freq = 25;
        } else if (val < 100 && val >= 50) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_50_HZ; /*(GYRO_CONFIG0_ODR_50_HZ)*/
                *freq = 50;
        } else if (val < 200 && val >= 100) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_100_HZ; /*(= GYRO_CONFIG0_ODR_100_HZ )*/
                *freq = 100;
        } else if (val < 400 && val >= 200) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_200_HZ; /*(= GYRO_CONFIG0_ODR_200_HZ )*/
                *freq = 200;
        } else if (val < 800 && val >= 400) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_400_HZ; /*(= GYRO_CONFIG0_ODR_400_HZ )*/
                *freq = 400;
        } else if (val < 1600 && val >= 800) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_800_HZ; /*(= GYRO_CONFIG0_ODR_800_HZ )*/
                *freq = 800;
        } else if (val == 1600) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_1600_HZ; /*(= GYRO_CONFIG0_ODR_1600_HZ )*/
                *freq = 1600;
        } else if (val == 3200) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_3200_HZ; /*(= GYRO_CONFIG0_ODR_3200_HZ )*/
                *freq = 3200;
        } else if (val == 6400) {
                odr_enum = ACCEL_CONFIG0_ACCEL_ODR_6400_HZ; /*(= GYRO_CONFIG0_ODR_6400_HZ )*/
                *freq = 6400;
        }

        return odr_enum;
}

static uint32_t convert_acc_fs_to_enum(uint32_t val, uint8_t *fs)
{
	uint32_t fs_enum = 0;

	if (val < 4 && val >= 2) {
		fs_enum = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G;
		*fs = 2;
	} else if (val < 8 && val >= 4) {
		fs_enum = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G;
		*fs = 4;
	} else if (val < 16 && val >= 8) {
		fs_enum = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G;
		*fs = 8;
	} else if (val == 16) {
		fs_enum = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G;
		*fs = 16;
	}
	return fs_enum;
}

static uint32_t convert_gyr_fs_to_enum(uint32_t val, uint16_t *fs)
{
        uint32_t fs_sel = GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS;

        if (val < 31 && val >= 15) {
                fs_sel = GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS;
                *fs = 15;
        } else if (val < 62 && val >= 31) {
                fs_sel = GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS;
                *fs = 31;
        } else if (val < 125 && val >= 62) {
                fs_sel = GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS;
                *fs = 62;
        } else if (val < 250 && val >= 125) {
                fs_sel = GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS;
                *fs = 125;
        } else if (val < 500 && val >= 250) {
                fs_sel = GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS;
                *fs = 250;
        } else if (val < 1000 && val >= 500) {
                fs_sel = GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS;
                *fs = 500;
        } else if (val < 2000 && val >= 1000) {
                fs_sel = GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS;
                *fs = 1000;
        } else if (val == 2000) {
                fs_sel = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;
                *fs = 2000;
        }

        return fs_sel;
}

uint32_t convert_ln_bw_to_enum(uint32_t val)
{
	uint32_t bw_enum = 0xFF;

	if (val < 8 && val >= 4) {
		bw_enum = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4; /* (= GIPREG_SYS2_REG_172_GYRO_UI_LPFBW_DIV_4) */
	} else if (val < 16 && val >= 8) {
		bw_enum = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_8; /* (= GIPREG_SYS2_REG_172_GYRO_UI_LPFBW_DIV_8) */
	} else if (val < 32 && val >= 16) {
		bw_enum = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_16; /* (= GIPREG_SYS2_REG_172_GYRO_UI_LPFBW_DIV_16) */
	} else if (val < 64 && val >= 32) {
		bw_enum = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_32; /* (= GIPREG_SYS2_REG_172_GYRO_UI_LPFBW_DIV_32) */
	} else if (val < 128 && val >= 64) {
		bw_enum = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_64; /* (= GIPREG_SYS2_REG_172_GYRO_UI_LPFBW_DIV_64) */
	} else if (val == 128) {
		bw_enum = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_128; /* (= GIPREG_SYS2_REG_172_GYRO_UI_LPFBW_DIV_128) */
	} else if (val == 0) {
		bw_enum = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_NO_FILTER;
		/*(= GIPREG_SYS2_REG_172_GYRO_UI_LPFBW_NO_FILTER)*/
	}
	return bw_enum;
}

static uint32_t convert_lp_avg_to_enum(uint32_t val)
{
	uint32_t avg_enum = 0xFF;

	if (val < 2 && val >= 1) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_1;
	} else if (val < 4 && val >= 2) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_2;
	} else if (val < 5 && val >= 4) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_4;
	} else if (val < 7 && val >= 5) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_5;
	} else if (val < 8 && val >= 7) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_7;
	} else if (val < 10 && val >= 8) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_8;
	} else if (val < 11 && val >= 10) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_10;
	} else if (val < 16 && val >= 11) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_11;
	} else if (val < 18 && val >= 16) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_16;
	} else if (val < 20 && val >= 18) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_18;
	} else if (val < 32 && val >= 20) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_20;
	} else if (val < 64 && val >= 32) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_32;
	} else if (val ==62) {
		avg_enum = IPREG_SYS2_REG_129_ACCEL_LP_AVG_64;
	}
	return avg_enum;
}

static uint8_t convert_enum_to_acc_fs(uint8_t fs_enum)
{
        uint8_t acc_fs = 0;

        if (fs_enum == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G) {
                acc_fs = 2;
        } else if (fs_enum == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G) {
                acc_fs = 4;
        } else if (fs_enum == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G) {
                acc_fs = 8;
        } else if (fs_enum == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G) {
                acc_fs = 16;
        }
        return acc_fs;
}

uint16_t convert_enum_to_gyr_fs(uint8_t fs_enum)
{
        uint16_t gyr_fs = 0;

	if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS) {
		gyr_fs = 15;
	} else if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS) {
		gyr_fs = 31;
	} else if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS) {
		gyr_fs = 62;
	} else if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS) {
		gyr_fs = 125;
	} else if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS) {
                gyr_fs = 250;
        } else if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS) {
                gyr_fs = 500;
        } else if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS) {
                gyr_fs = 1000;
        } else if (fs_enum == GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS) {
                gyr_fs = 2000;
        }
        return gyr_fs;
}


static int icm456xx_set_accel_power_mode(struct icm456xx_data *drv_data,
					 const struct sensor_value *val)
{
	if ((val->val1 == ICM456XX_LOW_POWER_MODE) &&
			(drv_data->accel_pwr_mode != ICM456XX_LOW_POWER_MODE)) {
		if (drv_data->accel_hz != 0) {
			if (drv_data->accel_hz <= 400) {
				inv_imu_set_accel_mode(&drv_data->driver, PWR_MGMT0_ACCEL_MODE_LP);

			} else {
				LOG_ERR("Not supported ATTR value");
				return -EINVAL;
			}
		}
		drv_data->accel_pwr_mode = val->val1;
	} else if ((val->val1 == ICM456XX_LOW_NOISE_MODE) &&
			(drv_data->accel_pwr_mode != ICM456XX_LOW_NOISE_MODE)) {
		if (drv_data->accel_hz != 0) {
			if (drv_data->accel_hz >= 12) {
				inv_imu_set_accel_mode(&drv_data->driver, PWR_MGMT0_ACCEL_MODE_LN);
			} else {
				LOG_ERR("Not supported ATTR value");
				return -EINVAL;
			}
		}
		drv_data->accel_pwr_mode = val->val1;
	} else {
		LOG_ERR("Not supported ATTR value");
		return -EINVAL;
	}
	return 0;
}

static int icm456xx_set_accel_odr(struct icm456xx_data *drv_data, const struct sensor_value *val)
{
	if (val->val1 <= 6400 && val->val1 >= 1) {
		if (drv_data->accel_hz == 0) {
			inv_imu_set_accel_frequency(&drv_data->driver,
					convert_freq_to_enum(val->val1, &drv_data->accel_hz));
			if (drv_data->accel_pwr_mode == ICM456XX_LOW_POWER_MODE) {
				inv_imu_set_accel_mode(&drv_data->driver, PWR_MGMT0_ACCEL_MODE_LP);

			} else if (drv_data->accel_pwr_mode == ICM456XX_LOW_NOISE_MODE) {
				inv_imu_set_accel_mode(&drv_data->driver, PWR_MGMT0_ACCEL_MODE_LN);
			}
		} else {
			inv_imu_set_accel_frequency(&drv_data->driver,
					convert_freq_to_enum(val->val1, &drv_data->accel_hz));
		}
	} else if (val->val1 == 0) {
		inv_imu_set_accel_mode(&drv_data->driver, PWR_MGMT0_ACCEL_MODE_OFF);
		drv_data->accel_hz = val->val1;
	} else {
		LOG_ERR("Incorrect sampling value");
		return -EINVAL;
	}
	return 0;
}

static int icm456xx_set_accel_fs(struct icm456xx_data *drv_data, const struct sensor_value *val)
{
	if (val->val1 > 16 || val->val1 < 2) {
		LOG_ERR("Incorrect fullscale value");
		return -EINVAL;
	}
	inv_imu_set_accel_fsr(&drv_data->driver,
			convert_acc_fs_to_enum(val->val1, &drv_data->accel_fs));
	LOG_DBG("Set accel full scale to: %d G", drv_data->accel_fs);
	return 0;

	return 0;
}

static int icm456xx_set_gyro_odr(struct icm456xx_data *drv_data, const struct sensor_value *val)
{
	if (val->val1 <= 6400 && val->val1 >= 12) {
		inv_imu_set_gyro_frequency(&drv_data->driver,
				convert_freq_to_enum(val->val1, &drv_data->gyro_hz));
		if (drv_data->gyro_hz == 0) 
			inv_imu_set_gyro_mode(&drv_data->driver, PWR_MGMT0_GYRO_MODE_LN);
	} else if (val->val1 == 0) {
		inv_imu_set_gyro_mode(&drv_data->driver, PWR_MGMT0_GYRO_MODE_OFF);
		drv_data->gyro_hz = val->val1;
	} else {
		LOG_ERR("Incorrect sampling value");
		return -EINVAL;
	}
	return 0;
}

static int icm456xx_set_gyro_fs(struct icm456xx_data *drv_data, const struct sensor_value *val)
{
	int32_t val_dps = sensor_rad_to_degrees(val);

	if (val_dps > 2000 || val_dps < 15) {
		LOG_ERR("Incorrect fullscale value");
		return -EINVAL;
	}
	inv_imu_set_gyro_fsr(&drv_data->driver,
			convert_gyr_fs_to_enum(val_dps, &drv_data->gyro_fs));
	LOG_DBG("Set gyro fullscale to: %d dps", drv_data->gyro_fs);
	return 0;
}

static int icm456xx_accel_config(struct icm456xx_data *drv_data, enum sensor_attribute attr,
				 const struct sensor_value *val)
{
	if (attr == SENSOR_ATTR_CONFIGURATION) {
		icm456xx_set_accel_power_mode(drv_data, val);

	} else if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
		icm456xx_set_accel_odr(drv_data, val);

	} else if (attr == SENSOR_ATTR_FULL_SCALE) {
		icm456xx_set_accel_fs(drv_data, val);

	} else if ((enum sensor_attribute_icm456xx)attr == SENSOR_ATTR_BW_FILTER_LPF) {
		if (val->val1 > 180) {
			LOG_ERR("Incorrect low pass filter bandwidth value");
			return -EINVAL;
		}
		inv_imu_set_accel_ln_bw(&drv_data->driver, convert_ln_bw_to_enum(val->val1));

	} else if ((enum sensor_attribute_icm456xx)attr == SENSOR_ATTR_AVERAGING) {
		if (val->val1 > 64 || val->val1 < 2) {
			LOG_ERR("Incorrect averaging filter value");
			return -EINVAL;
		}
		inv_imu_set_accel_lp_avg(&drv_data->driver, convert_lp_avg_to_enum(val->val1));
	} else {
		LOG_ERR("Unsupported attribute");
		return -EINVAL;
	}
	return 0;
}

static int icm456xx_gyro_config(struct icm456xx_data *drv_data, enum sensor_attribute attr,
				 const struct sensor_value *val)
{
	if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
		icm456xx_set_gyro_odr(drv_data, val);

	} else if (attr == SENSOR_ATTR_FULL_SCALE) {
		icm456xx_set_gyro_fs(drv_data, val);
	} else if ((enum sensor_attribute_icm456xx)attr == SENSOR_ATTR_BW_FILTER_LPF) {
		if (val->val1 > 180) {
			LOG_ERR("Incorrect low pass filter bandwidth value");
			return -EINVAL;
		}
		inv_imu_set_gyro_ln_bw(&drv_data->driver, convert_ln_bw_to_enum(val->val1));
	} else {
		LOG_ERR("Unsupported attribute");
		return -EINVAL;
	}
	return 0;
}

static int icm456xx_sensor_init(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	const struct icm456xx_config *config = dev->config;
	int err = 0;

	/* Initialize serial interface and device */
	data->driver.transport.context = (struct device *)dev;
	data->driver.transport.read_reg = inv_io_hal_read_reg;
	data->driver.transport.write_reg = inv_io_hal_write_reg;
	data->driver.transport.serif_type = config->serif_type;
	data->driver.transport.sleep_us = inv_sleep_us;

	if (data->driver.transport.serif_type == UI_SPI3 || data->driver.transport.serif_type == UI_SPI4) {
		drive_config0_t drive_config0;
		drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
		err |= inv_imu_write_reg(&data->driver, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
		inv_sleep_us(2); /* Takes effect 1.5 us after the register is programmed */
	}

	err = inv_imu_get_who_am_i(&data->driver, &data->chip_id);
	if (err < 0) {
		LOG_ERR("ID read failed: %d", err);
		return err;
	}

	if (data->chip_id != data->imu_whoami) {
		LOG_ERR("invalid WHO_AM_I value, was 0x%x but expected 0x%x for %s", data->chip_id,
			data->imu_whoami, data->imu_name);
		return -ENOTSUP;
	}
	err |= inv_imu_soft_reset(&data->driver);

	LOG_DBG("\"%s\" %s OK", dev->name, data->imu_name);
	return err;
}

static int icm456xx_turn_on_sensor(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	const struct icm456xx_config *cfg = dev->config;
	inv_imu_fifo_config_t    fifo_config;
	inv_imu_int_state_t            int_config;
	int err = 0;

	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
#ifdef CONFIG_ICM456XX_TRIGGER
	int_config.INV_FIFO_THS = INV_IMU_ENABLE;
#else
	int_config.INV_UI_DRDY = INV_IMU_ENABLE;
#endif
	err |= inv_imu_set_config_int(&data->driver, INV_IMU_INT1, &int_config);

#ifdef CONFIG_ICM456XX_TRIGGER
	// FIFO Config
	err |= inv_imu_get_fifo_config(&data->driver, &fifo_config);
	fifo_config.gyro_en    = INV_IMU_ENABLE;
	fifo_config.accel_en   = INV_IMU_ENABLE;
	fifo_config.hires_en   = INV_IMU_DISABLE;
	fifo_config.fifo_wm_th = 1;
	fifo_config.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT;
	err |= inv_imu_set_fifo_config(&data->driver, &fifo_config);
#endif

	err = inv_imu_set_accel_fsr(&data->driver, cfg->accel_fs);
	if (err < 0) {
		LOG_ERR("Failed to configure accel FSR");
		return -EIO;
	}
	data->accel_fs = convert_enum_to_acc_fs(cfg->accel_fs);

	err = inv_imu_set_gyro_fsr(&data->driver, cfg->gyro_fs);
	if ((err < 0)) {
		LOG_ERR("Failed to configure gyro FSR");
		return -EIO;
	}
	data->gyro_fs = convert_enum_to_gyr_fs(cfg->gyro_fs);

	err |= inv_imu_set_accel_ln_bw(&data->driver, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
	err |= inv_imu_set_gyro_ln_bw(&data->driver, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);

	if (cfg->accel_hz != 0) {
		err |= inv_imu_set_accel_frequency(
			&data->driver, cfg->accel_hz);
		if ((cfg->accel_pwr_mode == ICM456XX_LOW_NOISE_MODE) &&
		    (convert_dt_enum_to_freq(cfg->accel_hz) >= 12)) {
			err |= inv_imu_set_accel_mode(&data->driver, PWR_MGMT0_ACCEL_MODE_LP);
		} else if ((cfg->accel_pwr_mode == ICM456XX_LOW_POWER_MODE) &&
			   (convert_dt_enum_to_freq(cfg->accel_hz) <= 400)) {
			err |= inv_imu_set_accel_mode(&data->driver, PWR_MGMT0_ACCEL_MODE_LP);
		} else {
			LOG_ERR("Not supported power mode value");
		}
	}

	if (cfg->gyro_hz != 0) {
		err |= inv_imu_set_gyro_frequency(
				&data->driver, cfg->gyro_hz);
		err |= inv_imu_set_gyro_mode(&data->driver, PWR_MGMT0_GYRO_MODE_LP);
	}

	if (err < 0) {
		LOG_ERR("Failed to configure ODR.");
		return -EIO;
	}

	data->accel_pwr_mode = cfg->accel_pwr_mode;
	data->accel_hz = convert_dt_enum_to_freq(cfg->accel_hz);
	data->gyro_hz = convert_dt_enum_to_freq(cfg->gyro_hz);


	/*
	 * Accelerometer sensor need at least 10ms startup time
	 * Gyroscope sensor need at least 30ms startup time
	 */
	k_msleep(100);
	return 0;
}

static void icm456xx_convert_accel(struct sensor_value *val, int16_t raw_val, uint16_t fs)
{
	int64_t conv_val;

	/* 16 bit accelerometer. 2^15 bits represent the range in G */
	/* see datasheet section 3.2 for details */
	conv_val = (int64_t)raw_val * SENSOR_G * fs / INT16_MAX;

	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

void icm456xx_convert_gyro(struct sensor_value *val, int16_t raw_val, uint16_t fs)
{
        int64_t conv_val;

        /* 16 bit gyroscope. 2^15 bits represent the range in degrees/s */
        /* see datasheet section 3.1 for details */
        conv_val = ((int64_t)raw_val * fs * SENSOR_PI) / (INT16_MAX * 180U);

        val->val1 = conv_val / 1000000;
        val->val2 = conv_val % 1000000;
}

static void icm456xx_convert_temp(struct sensor_value *val, int16_t raw_val)
{
	int64_t conv_val;

	/* see datasheet section 15.9 for details */
	conv_val = 25 * 1000000 + ((int64_t)raw_val * 1000000 / 2);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

static int icm456xx_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	int res = 0;
	struct icm456xx_data *data = dev->data;

	icm456xx_lock(dev);
	if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		icm456xx_convert_accel(&val[0], data->accel_x, data->accel_fs);
		icm456xx_convert_accel(&val[1], data->accel_y, data->accel_fs);
		icm456xx_convert_accel(&val[2], data->accel_z, data->accel_fs);
	} else if (chan == SENSOR_CHAN_ACCEL_X) {
		icm456xx_convert_accel(val, data->accel_x, data->accel_fs);
	} else if (chan == SENSOR_CHAN_ACCEL_Y) {
		icm456xx_convert_accel(val, data->accel_y, data->accel_fs);
	} else if (chan == SENSOR_CHAN_ACCEL_Z) {
		icm456xx_convert_accel(val, data->accel_z, data->accel_fs);
	} else if (chan == SENSOR_CHAN_GYRO_XYZ) {
		icm456xx_convert_gyro(&val[0], data->gyro_x, data->gyro_fs);
		icm456xx_convert_gyro(&val[1], data->gyro_y, data->gyro_fs);
		icm456xx_convert_gyro(&val[2], data->gyro_z, data->gyro_fs);
	} else if (chan == SENSOR_CHAN_GYRO_X) {
		icm456xx_convert_gyro(val, data->gyro_x, data->gyro_fs);
	} else if (chan == SENSOR_CHAN_GYRO_Y) {
		icm456xx_convert_gyro(val, data->gyro_y, data->gyro_fs);
	} else if (chan == SENSOR_CHAN_GYRO_Z) {
		icm456xx_convert_gyro(val, data->gyro_z, data->gyro_fs);
	} else if (chan == SENSOR_CHAN_DIE_TEMP) {
		icm456xx_convert_temp(val, data->temp);
	} else {
		res = -ENOTSUP;
	}


	icm456xx_unlock(dev);

	return res;
}

#ifdef CONFIG_ICM456XX_TRIGGER
static int icm456xx_fetch_from_fifo(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	int status = 0;
	inv_imu_int_state_t int_state;
	inv_imu_fifo_data_t fd;

	status |= inv_imu_get_int_status(&data->driver, INV_IMU_INT1, &int_state);
	if (int_state.INV_FIFO_THS) {
		status |= inv_imu_get_fifo_frame(&data->driver, &fd);
		data->accel_x = fd.byte_16.accel_data[0];
		data->accel_y = fd.byte_16.accel_data[1];
		data->accel_z = fd.byte_16.accel_data[2];
		data->gyro_x = fd.byte_16.gyro_data[0];
		data->gyro_y = fd.byte_16.gyro_data[1];
		data->gyro_z = fd.byte_16.gyro_data[2];
		data->temp = (int16_t)fd.byte_16.temp_data;
	}
	return status;
}
#endif

#ifndef CONFIG_ICM456XX_TRIGGER
static int icm456xx_sample_fetch_accel(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	uint8_t buffer[ACCEL_DATA_SIZE];

	int res = inv_imu_read_reg(&data->driver, ACCEL_DATA_X1_UI, ACCEL_DATA_SIZE, buffer);
	
	if (res) {
		return res;
	}

	data->accel_x = (int16_t)sys_get_be16(&buffer[0]);
	data->accel_y = (int16_t)sys_get_be16(&buffer[2]);
	data->accel_z = (int16_t)sys_get_be16(&buffer[4]);

	return 0;
}

int icm45686_sample_fetch_gyro(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	uint8_t buffer[ACCEL_DATA_SIZE];

	int res = inv_imu_read_reg(&data->driver, GYRO_DATA_X1_UI, ACCEL_DATA_SIZE, buffer);
	
	if (res) {
		return res;
	}

	data->gyro_x = (int16_t)sys_get_be16(&buffer[0]);
	data->gyro_y = (int16_t)sys_get_be16(&buffer[2]);
	data->gyro_z = (int16_t)sys_get_be16(&buffer[4]);

        return 0;
}

static int icm456xx_sample_fetch_temp(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	uint8_t buffer[TEMP_DATA_SIZE];

	int res = inv_imu_read_reg(&data->driver, TEMP_DATA1_UI, TEMP_DATA_SIZE, buffer);
	
	if (res) {
		return res;
	}

	data->temp = (int16_t)sys_get_be16(&buffer[0]);

        return 0;
}

static int icm456xx_fetch_from_registers(const struct device *dev, enum sensor_channel chan)
{
	struct icm456xx_data *data = dev->data;
	int res = 0;
	int err = 0;
	inv_imu_int_state_t int_state;

	icm456xx_lock(dev);

	/* Ensure data ready status bit is set */
	err |= inv_imu_get_int_status(&data->driver, INV_IMU_INT1, &int_state);

	if (int_state.INV_UI_DRDY) {
		switch (chan) {
		case SENSOR_CHAN_ALL:
			err |= icm456xx_sample_fetch_accel(dev);
			err |= icm45686_sample_fetch_gyro(dev);
			err |= icm456xx_sample_fetch_temp(dev);
			break;
		case SENSOR_CHAN_ACCEL_XYZ:
		case SENSOR_CHAN_ACCEL_X:
		case SENSOR_CHAN_ACCEL_Y:
		case SENSOR_CHAN_ACCEL_Z:
			err |= icm456xx_sample_fetch_accel(dev);
			break;
		case SENSOR_CHAN_GYRO_XYZ:
		case SENSOR_CHAN_GYRO_X:
		case SENSOR_CHAN_GYRO_Y:
		case SENSOR_CHAN_GYRO_Z:
			err |= icm45686_sample_fetch_gyro(dev);
			break;
		case SENSOR_CHAN_DIE_TEMP:
			err |= icm456xx_sample_fetch_temp(dev);
			break;
		default:
			res = -ENOTSUP;
			break;
		}
	}

	icm456xx_unlock(dev);

	if (err < 0) {
		res = -EIO;
	}
	return res;
}
#endif

static int icm456xx_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int status = 0;

	icm456xx_lock(dev);
#ifdef CONFIG_TDK_APEX
	if ((enum sensor_channel_tdk_apex)chan == SENSOR_CHAN_APEX_MOTION) {
		status = icm456xx_apex_fetch_from_dmp(dev);
	}
#endif

	if ((chan == SENSOR_CHAN_ALL) || SENSOR_CHANNEL_IS_ACCEL(chan) ||
	    (SENSOR_CHANNEL_IS_GYRO(chan)) ||
	    (chan == SENSOR_CHAN_DIE_TEMP)) {
#ifdef CONFIG_ICM456XX_TRIGGER
		status = icm456xx_fetch_from_fifo(dev);
#else
		status = icm456xx_fetch_from_registers(dev, chan);
#endif
	}

	icm456xx_unlock(dev);
	return status;
}

static int icm456xx_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	struct icm456xx_data *drv_data = dev->data;

	__ASSERT_NO_MSG(val != NULL);

	icm456xx_lock(dev);

	if ((enum sensor_channel_tdk_apex)chan == SENSOR_CHAN_APEX_MOTION) {
		if (attr == SENSOR_ATTR_CONFIGURATION) {
#ifdef CONFIG_TDK_APEX
			if (val->val1 == TDK_APEX_PEDOMETER) {
				icm456xx_apex_enable(&drv_data->driver);
				icm456xx_apex_enable_pedometer(dev, &drv_data->driver);
			} else if (val->val1 == TDK_APEX_TILT) {
				icm456xx_apex_enable(&drv_data->driver);
				icm456xx_apex_enable_tilt(&drv_data->driver);
			} else if (val->val1 == TDK_APEX_SMD) {
				icm456xx_apex_enable(&drv_data->driver);
				icm456xx_apex_enable_smd(&drv_data->driver);
			} else if (val->val1 == TDK_APEX_WOM) {
				icm456xx_apex_enable_wom(&drv_data->driver);
			} else {
				LOG_ERR("Not supported ATTR value");
			}
#endif
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}
	} else if (SENSOR_CHANNEL_IS_ACCEL(chan)) {
		icm456xx_accel_config(drv_data, attr, val);
#if CONFIG_USE_EMD_ICM45686
	} else if (SENSOR_CHANNEL_IS_GYRO(chan)) {
		icm456xx_gyro_config(drv_data, attr, val);
#endif
	} else {
		LOG_ERR("Unsupported channel");
		(void)drv_data;
		return -EINVAL;
	}

	icm456xx_unlock(dev);

	return 0;
}

static int icm456xx_attr_get(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	const struct icm456xx_data *data = dev->data;
	const struct icm456xx_config *cfg = dev->config;
	int res = 0;

	__ASSERT_NO_MSG(val != NULL);

	icm456xx_lock(dev);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			val->val1 = data->accel_hz;
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			val->val1 = data->accel_fs;
		} else {
			LOG_ERR("Unsupported attribute");
			res = -EINVAL;
		}
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			val->val1 = data->gyro_hz;
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			val->val1 = data->gyro_fs;
		} else {
			LOG_ERR("Unsupported attribute");
			res = -EINVAL;
		}
		break;
	case SENSOR_CHAN_APEX_MOTION:
		if (attr == SENSOR_ATTR_CONFIGURATION) {
			val->val1 = cfg->apex;
		}
		break;

	default:
		LOG_ERR("Unsupported channel");
		res = -EINVAL;
		break;
	}

	icm456xx_unlock(dev);

	return res;
}

static inline int icm456xx_bus_check(const struct device *dev)
{
	const struct icm456xx_config *cfg = dev->config;

	return cfg->bus_io->check(&cfg->bus);
}

static int icm456xx_init(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	int res = 0;

	if (icm456xx_bus_check(dev) < 0) {
		LOG_ERR("bus check failed");
		return -ENODEV;
	}

	data->accel_x = 0;
	data->accel_y = 0;
	data->accel_z = 0;
	data->gyro_x = 0;
	data->gyro_y = 0;
	data->gyro_z = 0;
	data->temp = 0;

	if (icm456xx_sensor_init(dev)) {
		LOG_ERR("could not initialize sensor");
		return -EIO;
	}

#ifdef CONFIG_ICM456XX_TRIGGER
	res |= icm456xx_trigger_enable_interrupt(dev);
	res |= icm456xx_trigger_init(dev);
	if (res < 0) {
		LOG_ERR("Failed to initialize interrupt.");
		return res;
	}
#endif

	res |= icm456xx_turn_on_sensor(dev);

	return res;
}

#ifndef CONFIG_ICM456XX_TRIGGER

void icm456xx_lock(const struct device *dev)
{
	ARG_UNUSED(dev);
}

void icm456xx_unlock(const struct device *dev)
{
	ARG_UNUSED(dev);
}

#endif

static DEVICE_API(sensor, icm456xx_driver_api) = {
#ifdef CONFIG_ICM456XX_TRIGGER
	.trigger_set = icm456xx_trigger_set,
#endif
	.sample_fetch = icm456xx_sample_fetch,
	.channel_get = icm456xx_channel_get,
	.attr_set = icm456xx_attr_set,
	.attr_get = icm456xx_attr_get,
};

/* device defaults to spi mode 0/3 support */
#define ICM456XX_SPI_CFG (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA)

/* Initializes a common struct icm456xx_config */
#define ICM456XX_CONFIG_COMMON(inst)                                                               \
	IF_ENABLED(CONFIG_ICM456XX_TRIGGER,	\
			(.gpio_int = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),))     \
			.accel_fs = DT_INST_ENUM_IDX(inst, accel_fs),	\
			.accel_hz = DT_INST_ENUM_IDX(inst, accel_hz),	\
			.accel_pwr_mode = DT_INST_ENUM_IDX(inst, power_mode),	\
			.accel_avg = DT_INST_ENUM_IDX(inst, accel_avg),	\
			.accel_filt_bw = DT_INST_ENUM_IDX(inst, accel_filt_bw_hz),	\
			.apex = DT_INST_ENUM_IDX(inst, apex),                          \
			.gyro_fs = DT_INST_ENUM_IDX(inst, gyro_fs),                     \
			.gyro_hz = DT_INST_ENUM_IDX(inst, gyro_hz),                     \
			.gyro_filt_bw = DT_INST_ENUM_IDX(inst, gyro_filt_bw_hz)

/* Initializes the bus members for an instance on a SPI bus. */
#define ICM456XX_CONFIG_SPI(inst)                                                                  \
	{.bus.spi = SPI_DT_SPEC_INST_GET(inst, ICM456XX_SPI_CFG, 0),                               \
	 .bus_io = &icm456xx_bus_io_spi,                                                           \
	 .serif_type = UI_SPI4,                                                                    \
	 ICM456XX_CONFIG_COMMON(inst)}

/* Initializes the bus members for an instance on an I2C bus. */
#define ICM456XX_CONFIG_I2C(inst)                                                                  \
	{.bus.i2c = I2C_DT_SPEC_INST_GET(inst),                                                    \
	 .bus_io = &icm456xx_bus_io_i2c,                                                           \
	 .serif_type = UI_I2C,                                                                     \
	 ICM456XX_CONFIG_COMMON(inst)}

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define ICM456XX_DEFINE(inst, name, whoami)                                                        \
	static struct icm456xx_data icm456xx_data_##inst = {                                       \
		.imu_name = name,                                                                  \
		.imu_whoami = whoami,                                                              \
	};                                                                                         \
	static const struct icm456xx_config icm456xx_config_##inst = \
			COND_CODE_1(DT_INST_ON_BUS(inst, spi),	\
			(ICM456XX_CONFIG_SPI(inst)),	\
			(ICM456XX_CONFIG_I2C(inst)));             \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icm456xx_init, NULL, &icm456xx_data_##inst,             \
				     &icm456xx_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &icm456xx_driver_api);

#define DT_DRV_COMPAT invensense_icm45686
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY_VARGS(ICM456XX_DEFINE, INV_IMU_STRING_ID, INV_IMU_WHOAMI);
#endif
#undef DT_DRV_COMPAT
