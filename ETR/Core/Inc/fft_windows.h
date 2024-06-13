#ifndef FFT_WINDOWS_H
#define FFT_WINDOWS_H

#include <stdint.h>
#include "arm_math.h"
#include "arm_const_structs.h"

void fft_hamming_f32(
        float32_t * pDst,
        uint32_t blockSize);

void fft_hanning_f32(
        float32_t * pDst,
        uint32_t blockSize);

void fft_bartlett_f32(
        float32_t * pDst,
        uint32_t blockSize);

void fft_blackman_harris_92db_f32(
        float32_t * pDst,
        uint32_t blockSize);

#endif // FFT_WINDOWS_H