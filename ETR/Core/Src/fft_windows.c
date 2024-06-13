#include "fft_windows.h"
void fft_hanning_f32(
        float32_t * pDst,
        uint32_t blockSize)
{
   float32_t k = 2.0f / ((float32_t) blockSize);
   float32_t w;

   for(uint32_t i=0;i<blockSize;i++)
   {
     w = PI * i * k;
     w = 0.5f * (1.0f - cosf (w));
     pDst[i] = w*pDst[i];
   }
}


void fft_hamming_f32(
        float32_t * pDst,
        uint32_t blockSize)
{
   float32_t k = 2.0f / ((float32_t) blockSize);
   float32_t w;

   for(uint32_t i=0;i<blockSize;i++)
   {
     w = 0.54f - 0.46f * cosf (PI * i * k);
     pDst[i] = w*pDst[i];
   }
}


void fft_bartlett_f32(
        float32_t * pDst,
        uint32_t blockSize)
{
   float32_t k = 2.0f / ((float32_t) blockSize);
   float32_t w;

   for(uint32_t i=0;i<blockSize;i++)
   {
     w = i * k ;
     if (i * k > 1.0f)
     {
       w = 2.0f - w;
     }
     pDst[i] = w*pDst[i];
   }
}


void fft_blackman_harris_92db_f32(
        float32_t * pDst,
        uint32_t blockSize)
{
   float32_t k = 2.0f / ((float32_t) blockSize);
   float32_t w;

   for(uint32_t i=0;i<blockSize;i++)
   {
     w = PI * i * k;
        w = 0.35875f - 0.48829f * cosf (w) +
    0.14128f * cosf (2.f * w) - 0.01168f * cosf (3.f * w);

        pDst[i] = w*pDst[i];
   }
}