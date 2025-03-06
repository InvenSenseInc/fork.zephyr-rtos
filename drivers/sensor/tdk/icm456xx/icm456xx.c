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
#include "icm45686.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ICM456XX, CONFIG_SENSOR_LOG_LEVEL);

/* Convert DT enum to sensor ODR selection */
#define ICM456XX_CONVERT_ENUM_TO_ODR_POS (4)

/* Maximum bytes to read/write on ICM456XX serial interface */
#define ICM456XX_SERIAL_INTERFACE_MAX_READ  (1024 * 32)
#define ICM456XX_SERIAL_INTERFACE_MAX_WRITE (1024 * 32)

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
		freq = 1600;
		break;
	case 2:
		freq = 800;
		break;
	case 3:
		freq = 400;
		break;
	case 4:
		freq = 200;
		break;
	case 5:
		freq = 100;
		break;
	case 6:
		freq = 50;
		break;
	case 7:
		freq = 25;
		break;
	case 8:
		freq = 12;
		break;
	case 9:
		freq = 6;
		break;
	case 10:
		freq = 3;
		break;
	case 11:
		freq = 1;
		break;
	default:
		freq = 0;
		break;
	}
	return freq;
}

uint32_t convert_freq_to_bitfield(uint32_t val, uint16_t *freq)
{
	uint32_t odr_bitfield = 0;

	//TODO implementation
	return odr_bitfield;
}

static uint32_t convert_acc_fs_to_bitfield(uint32_t val, uint8_t *fs)
{
	uint32_t bitfield = 0;

	//TODO Implementation
	return bitfield;
}

uint32_t convert_ln_bw_to_bitfield(uint32_t val)
{
	uint32_t bitfield = 0xFF;

	//TODO Implementation
	return bitfield;
}

static uint32_t convert_lp_avg_to_bitfield(uint32_t val)
{
	uint32_t bitfield = 0xFF;

	// TODO Implementation
	return bitfield;
}

static uint8_t convert_bitfield_to_acc_fs(uint8_t bitfield)
{
	uint8_t acc_fs = 0;

	//TODO Implementation
	return acc_fs;
}

static int icm456xx_set_accel_power_mode(struct icm456xx_data *drv_data,
					 const struct sensor_value *val)
{
	//TODO Implementation
	return 0;
}

static int icm456xx_set_accel_odr(struct icm456xx_data *drv_data, const struct sensor_value *val)
{
	return 0;
}

static int icm456xx_set_accel_fs(struct icm456xx_data *drv_data, const struct sensor_value *val)
{
	return 0;
}

static int icm456xx_accel_config(struct icm456xx_data *drv_data, enum sensor_attribute attr,
				 const struct sensor_value *val)
{
	LOG_ERR("accel config");
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
	LOG_ERR("WHO %X, whoami %X", err, data->chip_id);
	if (err < 0) {
		LOG_ERR("ID read failed: %d", err);
		return err;
	}

	LOG_ERR("IMU NAME %s A fs %d G fs %d A Hz %d, G Hz %d", data->imu_name, config->accel_fs, config->gyro_fs, config->accel_hz, config->gyro_hz);
	if (data->chip_id != data->imu_whoami) {
		LOG_ERR("invalid WHO_AM_I value, was 0x%x but expected 0x%x for %s", data->chip_id,
			data->imu_whoami, data->imu_name);
		return -ENOTSUP;
	}
	err |= inv_imu_soft_reset(&data->driver);


	LOG_DBG("\"%s\" %s OK", dev->name, data->imu_name);
	return 0;
}

static int icm456xx_turn_on_sensor(const struct device *dev)
{
	struct icm456xx_data *data = dev->data;
	const struct icm456xx_config *cfg = dev->config;
	inv_imu_adv_fifo_config_t fifo_config;
	inv_imu_int_state_t            int_config;
	int err = 0;

	/* Interrupts configuration:
	 * - Enable EDMP interrupt
	 * - Enable FIFO interrupt for GRV
	 * - Enable DRDY interrupt for raw data (raw data could also be retrieved from FIFO)
	 */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_UI_DRDY = INV_IMU_ENABLE;
	err |= inv_imu_set_config_int(&data->driver, INV_IMU_INT1, &int_config);

	LOG_ERR("fsr in turn on %d", cfg->accel_fs);
	err = inv_imu_set_accel_fsr(&data->driver,ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G);
	LOG_ERR("fsr in turn on %d, err %d", cfg->accel_fs, err);

	if (err < 0) {
		LOG_ERR("Failed to configure accel FSR");
		return -EIO;
	}
	LOG_ERR("Set accel full scale to: %d G", 0);

	err = inv_imu_set_gyro_fsr(&data->driver,GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);
	data->gyro_fs = convert_enum_to_gyr_fs(cfg->gyro_fs);
	if ((err < 0)) {
		LOG_ERR("Failed to configure gyro FSR");
		return -EIO;
	}
	LOG_ERR("Set gyro full scale to: %d dps", 0);

	err |= inv_imu_set_accel_ln_bw(&data->driver, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
	err |= inv_imu_set_gyro_ln_bw(&data->driver, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);

	if (cfg->accel_hz != 0) {
		LOG_ERR("accel freequency setup %d", cfg->accel_hz);

		err |= inv_imu_set_accel_frequency(
			&data->driver, ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ);
		if ((cfg->accel_pwr_mode == ICM456XX_LOW_NOISE_MODE) &&
		    (convert_dt_enum_to_freq(cfg->accel_hz) >= 12)) {
			LOG_ERR("set low noise mode");
			err |= inv_imu_set_accel_mode(&data->driver, PWR_MGMT0_ACCEL_MODE_LP);
		} else if ((cfg->accel_pwr_mode == ICM456XX_LOW_POWER_MODE) &&
			   (convert_dt_enum_to_freq(cfg->accel_hz) <= 400)) {
			LOG_ERR("set low power mode");
			err |= inv_imu_adv_enable_accel_lp(&data->driver);
			err |= inv_imu_set_accel_mode(&data->driver, PWR_MGMT0_ACCEL_MODE_LP);
		} else {
			LOG_ERR("Not supported power mode value");
		}
	}


	if (cfg->gyro_hz != 0) {
		err |= inv_imu_set_gyro_frequency(
				&data->driver, GYRO_CONFIG0_GYRO_ODR_6_25_HZ);
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
	LOG_ERR("turn on done err %d", err);
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
#ifdef CONFIG_TDK_APEX
	const struct icm456xx_config *cfg = dev->config;
#endif

	LOG_ERR("channel_get");
	icm456xx_lock(dev);
	if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		icm456xx_convert_accel(&val[0], data->accel_x, 8);
		icm456xx_convert_accel(&val[1], data->accel_y, 8);
		icm456xx_convert_accel(&val[2], data->accel_z, 8/*data->accel_fs*/);
	} else if (chan == SENSOR_CHAN_ACCEL_X) {
		icm456xx_convert_accel(val, data->accel_x, 8);
	} else if (chan == SENSOR_CHAN_ACCEL_Y) {
		icm456xx_convert_accel(val, data->accel_y, 8);
	} else if (chan == SENSOR_CHAN_ACCEL_Z) {
		icm456xx_convert_accel(val, data->accel_z, 8);
	} else if (chan == SENSOR_CHAN_GYRO_XYZ) {
		icm456xx_convert_gyro(&val[0], data->gyro_x, 2000);
		icm456xx_convert_gyro(&val[1], data->gyro_y, 2000);
		icm456xx_convert_gyro(&val[2], data->gyro_z, 2000);
	} else if (chan == SENSOR_CHAN_GYRO_X) {
		icm456xx_convert_gyro(val, data->gyro_x, 2000);
	} else if (chan == SENSOR_CHAN_GYRO_Y) {
		icm456xx_convert_gyro(val, data->gyro_y, 2000);
	} else if (chan == SENSOR_CHAN_GYRO_Z) {
		icm456xx_convert_gyro(val, data->gyro_z, 2000);
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
	inv_imu_sensor_data_t d;
	uint8_t input_mask = 0;

	uint16_t packet_size = FIFO_HEADER_SIZE + ACCEL_DATA_SIZE + GYRO_DATA_SIZE +
			       TEMP_DATA_SIZE + FIFO_TS_FSYNC_SIZE;
	uint16_t fifo_idx = 0;

	float                 accel_g[3];
	float                 gyro_dps[3];

	float                 temp_degc;

	LOG_ERR("fetch from fifo");
	status |= inv_imu_get_int_status(&data->driver, INV_IMU_INT1, &int_state);

	if (int_state.INV_UI_DRDY) {
		status |= inv_imu_get_register_data(&data->driver, &d);
		accel_g[0]  = (float)(d.accel_data[0] * 8 /* gee */) / 32768;
		accel_g[1]  = (float)(d.accel_data[1] * 8 /* gee */) / 32768;
		accel_g[2]  = (float)(d.accel_data[2] * 8 /* gee */) / 32768;
		gyro_dps[0] = (float)(d.gyro_data[0] * 2000 /* dps */) / 32768;
		gyro_dps[1] = (float)(d.gyro_data[1] * 2000 /* dps */) / 32768;
		gyro_dps[2] = (float)(d.gyro_data[2] * 2000 /* dps */) / 32768;
		temp_degc   = (float)25 + ((float)d.temp_data / 128);

		LOG_ERR("%f %f %f  / %f %f %f / %f", accel_g[0], accel_g[1], accel_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2], temp_degc);
		data->accel_x = d.accel_data[0];
		data->accel_y = d.accel_data[1];
		data->accel_z = d.accel_data[2];

		data->gyro_x = d.gyro_data[0];
		data->gyro_y = d.gyro_data[1];
		data->gyro_z = d.gyro_data[2];

		data->temp = d.temp_data;

	}

	return 0;
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

	LOG_ERR("fetch from accel");
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
	uint8_t buffer[ACCEL_DATA_SIZE];

	int res = inv_imu_read_reg(&data->driver, GYRO_DATA_X1_UI, ACCEL_DATA_SIZE, buffer);
	
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
        inv_imu_sensor_data_t d;

	LOG_DBG("Fetch from reg");

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
#if CONFIG_USE_EMD_ICM45686
	struct icm456xx_data *data = dev->data;
#endif
	int status = 0;

	uint8_t                  reg_data[ACCEL_DATA_SIZE + GYRO_DATA_SIZE + TEMP_DATA_SIZE];
	LOG_ERR("sample fetch");

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

//	status |= inv_imu_read_reg(&data->driver, ACCEL_DATA_X1_UI, sizeof(reg_data), reg_data);
//	LOG_ERR("data %X %X %X %X %X %X / %X %X %X %X %X %X ", reg_data[0], reg_data[1], reg_data[2], reg_data[3], reg_data[4], reg_data[5],  reg_data[6], reg_data[7], reg_data[8], reg_data[9], reg_data[10], reg_data[11]);

	icm456xx_unlock(dev);
	LOG_ERR("sample fetch done");
	return status;
}

static int icm456xx_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	struct icm456xx_data *drv_data = dev->data;

	__ASSERT_NO_MSG(val != NULL);

	icm456xx_lock(dev);

	LOG_ERR("attr_ set");
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
		icm45686_gyro_config(drv_data, attr, val);
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

	LOG_ERR("attr_get");
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
				    .accel_fs = DT_INST_ENUM_IDX(inst, accel_fs),                  \
				    .accel_hz = DT_INST_ENUM_IDX(inst, accel_hz),                  \
				    .apex = DT_INST_ENUM_IDX(inst, apex),                          \
		 IF_ENABLED(CONFIG_USE_EMD_ICM45686,                                       \
				(.gyro_fs = DT_INST_ENUM_IDX(inst, gyro_fs),))                     \
		 IF_ENABLED(CONFIG_USE_EMD_ICM45686,                                       \
				(.gyro_hz = DT_INST_ENUM_IDX(inst, gyro_hz),))                     \
		 IF_ENABLED(CONFIG_USE_EMD_ICM45686,                                       \
				(.gyro_filt_bw = DT_INST_ENUM_IDX(inst, gyro_filt_bw_hz),))

/* Initializes the bus members for an instance on a SPI bus. */
#define ICM456XX_CONFIG_SPI(inst)                                                                  \
	{.bus.spi = SPI_DT_SPEC_INST_GET(inst, ICM456XX_SPI_CFG, 0),                               \
	 .bus_io = &icm456xx_bus_io_spi,                                                           \
	 .serif_type = UI_SPI3,                                                                    \
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
DT_INST_FOREACH_STATUS_OKAY_VARGS(ICM456XX_DEFINE, /*INV_ICM45686_STRING_ID*/"ICM45686", 0xEE /*INV_IMU_WHOAMI*/);
#endif
#undef DT_DRV_COMPAT
