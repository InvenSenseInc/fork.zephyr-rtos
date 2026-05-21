#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>


#include "invn_algo_gaf.h"
#include <math.h>

LOG_MODULE_REGISTER(invn_algo, LOG_LEVEL_DBG);

static invn_algo_gaf_struct invn_algo_gaf;
invn_algo_gaf_input algo_gaf_input;
invn_algo_gaf_output algo_gaf_output;


int invn_algo_init(int odr_hz)
{
    invn_algo_gaf_config algo_gaf_config;
    int32_t odr_us = 1000000/odr_hz;
    int32_t gaf_sensor_odr_us[3] = {odr_us, odr_us, odr_us};

    invn_algo_gaf_generate_config(&algo_gaf_config, gaf_sensor_odr_us);
    algo_gaf_config.acc_fsr = 8;
    algo_gaf_config.gyr_fsr = 2000;
    algo_gaf_config.mag_sc_q16 = 2000*65536/32768; // +/-2000µT FSR over 16 bits
    algo_gaf_config.acc_pdr_us = odr_us;
    algo_gaf_config.acc_pdr_us = odr_us;
    algo_gaf_config.acc_pdr_us = odr_us;
    algo_gaf_config.acc_odr_us = odr_us;
    algo_gaf_config.gyr_odr_us = odr_us;
    algo_gaf_config.mag_odr_us = odr_us;
    algo_gaf_config.raw_data_format_flag = 0;
    algo_gaf_config.enable_acc_bias_flag = 1;
    algo_gaf_config.enable_gyr_bias_flag = 2;
    algo_gaf_config.enable_mag_bias_flag = 2;
    int8_t AlgoStatus = invn_algo_gaf_init(&invn_algo_gaf, NULL, &algo_gaf_config);
    LOG_ERR("gaf Init Status=%d",AlgoStatus);
    return AlgoStatus;
}

int invn_algo_set_odr(int odr_hz)
{
    invn_algo_gaf_odr_config gaf_odr_config;
    int32_t odr_us = 1000000/odr_hz;

    gaf_odr_config.acc_odr_us = odr_us;
    gaf_odr_config.gyr_odr_us = odr_us;
    gaf_odr_config.mag_odr_us = odr_us;
    gaf_odr_config.acc_pdr_us = odr_us;
    gaf_odr_config.gyr_pdr_us = odr_us;
    gaf_odr_config.mag_pdr_us = odr_us;
    invn_algo_gaf_set_odr(&invn_algo_gaf, &gaf_odr_config);
    return 0;
}

static void fixedpoint_to_float(const int32_t *in, float *out, const uint8_t fxp_shift,
                                const uint8_t dim)
{
    int   i;
    float scale = 1.f / (1 << fxp_shift);

    for (i = 0; i < dim; i++)
        out[i] = scale * in[i];
}

int invn_algo_process(int64_t time_us, int16_t * acc_raw, int16_t * gyr_raw, int16_t * mag_raw, float * quat, uint8_t * accuracy)
{
    invn_algo_gaf_input algo_gaf_input;
    invn_algo_gaf_output algo_gaf_output;
    algo_gaf_input.sRacc_data[0] = acc_raw[0];
    algo_gaf_input.sRacc_data[1] = acc_raw[1];
    algo_gaf_input.sRacc_data[2] = acc_raw[2];
    algo_gaf_input.sRgyr_data[0] = gyr_raw[0];
    algo_gaf_input.sRgyr_data[1] = gyr_raw[1];
    algo_gaf_input.sRgyr_data[2] = gyr_raw[2];
    algo_gaf_input.sRmag_data[0] = mag_raw[0];
    algo_gaf_input.sRmag_data[1] = mag_raw[1];
    algo_gaf_input.sRmag_data[2] = mag_raw[2];
    algo_gaf_input.sRimu_time_us = time_us;
    algo_gaf_input.sRtemp_data = 0;
    algo_gaf_input.mask = INVN_ALGO_GAF_INPUT_MASK_ACC | INVN_ALGO_GAF_INPUT_MASK_GYR | INVN_ALGO_GAF_INPUT_MASK_MAG;
    invn_algo_gaf_process(&invn_algo_gaf, &algo_gaf_input, &algo_gaf_output);
    if (algo_gaf_output.mask & INVN_ALGO_GAF_OUTPUT_MASK_QUAT_AGM)
    {
        fixedpoint_to_float(algo_gaf_output.rv_quat_q30, quat, 30, 4);
        accuracy[0] = algo_gaf_output.acc_accuracy_flag;
        accuracy[1] = algo_gaf_output.gyr_accuracy_flag;
        accuracy[2] = algo_gaf_output.mag_accuracy_flag;
    }
    return 0;
}

