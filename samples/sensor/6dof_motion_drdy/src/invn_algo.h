#ifndef INVN_ALGO_H_
#define INVN_ALGO_H_

// -------------------------------------------------

#include <stdbool.h>

// -------------------------------------------------


int invn_algo_init(int odr_hz);
int invn_algo_set_odr(int odr_hz);

int invn_algo_process(int64_t time_us, int16_t * acc_raw, int16_t * gyr_raw, int16_t * mag_raw, float * quat, uint8_t * accuracy);


#endif // INVN_ALGO_H_
