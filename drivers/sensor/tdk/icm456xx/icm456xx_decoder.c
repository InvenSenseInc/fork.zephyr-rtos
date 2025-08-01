/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/sensor_clock.h>

#include "icm456xx.h"
#include "icm456xx_decoder.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ICM456XX_DECODER, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT invensense_icm456xx

int icm456xx_convert_raw_to_q31(struct icm456xx_encoded_data *edata,
				enum sensor_channel chan,
				int32_t reading,
				q31_t *out)
{
	//TODO : Implementing
	return 0;
}


static int icm456xx_get_channel_position(enum sensor_channel chan)
{
        switch (chan) {
        case SENSOR_CHAN_ACCEL_XYZ:
        case SENSOR_CHAN_ACCEL_X:
                return offsetof(struct icm456xx_encoded_payload, accel.x) / sizeof(int16_t);
        case SENSOR_CHAN_ACCEL_Y:
                return offsetof(struct icm456xx_encoded_payload, accel.y) / sizeof(int16_t);
        case SENSOR_CHAN_ACCEL_Z:
                return offsetof(struct icm456xx_encoded_payload, accel.z) / sizeof(int16_t);
        case SENSOR_CHAN_GYRO_XYZ:
        case SENSOR_CHAN_GYRO_X:
                return offsetof(struct icm456xx_encoded_payload, gyro.x) / sizeof(int16_t);
        case SENSOR_CHAN_GYRO_Y:
                return offsetof(struct icm456xx_encoded_payload, gyro.y) / sizeof(int16_t);
        case SENSOR_CHAN_GYRO_Z:
                return offsetof(struct icm456xx_encoded_payload, gyro.z) / sizeof(int16_t);
        case SENSOR_CHAN_DIE_TEMP:
                return offsetof(struct icm456xx_encoded_payload, temp) / sizeof(int16_t);
        default:
                return 0;
        }
}

static uint8_t icm456xx_encode_channel(enum sensor_channel chan)
{
	        uint8_t encode_bmask = 0;

        switch (chan) {
        case SENSOR_CHAN_ACCEL_X:
        case SENSOR_CHAN_ACCEL_Y:
        case SENSOR_CHAN_ACCEL_Z:
        case SENSOR_CHAN_GYRO_X:
        case SENSOR_CHAN_GYRO_Y:
        case SENSOR_CHAN_GYRO_Z:
        case SENSOR_CHAN_DIE_TEMP:
                encode_bmask = BIT(icm456xx_get_channel_position(chan));
                break;
        case SENSOR_CHAN_ACCEL_XYZ:
                encode_bmask = BIT(icm456xx_get_channel_position(SENSOR_CHAN_ACCEL_X)) |
                               BIT(icm456xx_get_channel_position(SENSOR_CHAN_ACCEL_Y)) |
                               BIT(icm456xx_get_channel_position(SENSOR_CHAN_ACCEL_Z));
                break;
        case SENSOR_CHAN_GYRO_XYZ:
                encode_bmask = BIT(icm456xx_get_channel_position(SENSOR_CHAN_GYRO_X)) |
                               BIT(icm456xx_get_channel_position(SENSOR_CHAN_GYRO_Y)) |
                               BIT(icm456xx_get_channel_position(SENSOR_CHAN_GYRO_Z));
                break;
        default:
                break;
        }

        return encode_bmask;
}

int icm456xx_encode(const struct device *dev,
		    const struct sensor_chan_spec *const channels,
		    const size_t num_channels,
		    uint8_t *buf)
{
	struct icm456xx_encoded_data *edata = (struct icm456xx_encoded_data *)buf;
	const struct icm456xx_config *dev_config = dev->config;
	uint64_t cycles;
	int err;

	edata->header.channels = 0;

	for (size_t i = 0 ; i < num_channels ; i++) {
		edata->header.channels |= icm456xx_encode_channel(channels[i].chan_type);
	}

	err = sensor_clock_get_cycles(&cycles);
	if (err != 0) {
		return err;
	}

	edata->header.events = 0;
	edata->header.accel_fs = dev_config->accel_fs;
	edata->header.gyro_fs = dev_config->gyro_fs;
	edata->header.timestamp = sensor_clock_cycles_to_ns(cycles);

	return 0;
}

static int icm456xx_decoder_get_frame_count(const uint8_t *buffer,
					    struct sensor_chan_spec chan_spec,
					    uint16_t *frame_count)
{
	//TODO : Implementing, read from TDK HAL?
	return 0;
}

static int icm456xx_decoder_get_size_info(struct sensor_chan_spec chan_spec,
					  size_t *base_size,
					  size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_GYRO_XYZ:
		*base_size = sizeof(struct sensor_three_axis_data);
		*frame_size = sizeof(struct sensor_three_axis_sample_data);
		return 0;
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_DIE_TEMP:
		*base_size = sizeof(struct sensor_q31_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int icm456xx_decoder_decode(const uint8_t *buffer,
				   struct sensor_chan_spec chan_spec,
				   uint32_t *fit,
				   uint16_t max_count,
				   void *data_out)
{
	return 0;
}

static bool icm456xx_decoder_has_trigger(const uint8_t *buffer, enum sensor_trigger_type trigger)
{
	//TODO : Implementing, read from register is works
	return 0;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = icm456xx_decoder_get_frame_count,
	.get_size_info = icm456xx_decoder_get_size_info,
	.decode = icm456xx_decoder_decode,
	.has_trigger = icm456xx_decoder_has_trigger,
};

int icm456xx_get_decoder(const struct device *dev,
			 const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}
