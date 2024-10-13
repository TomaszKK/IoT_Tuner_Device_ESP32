#ifndef Yin_h
#define Yin_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define YIN_SAMPLING_RATE 44000
#define YIN_DEFAULT_THRESHOLD 0.10

typedef struct _Yin {
    int16_t bufferSize;
    int16_t halfBufferSize;
    float* yinBuffer;
    float probability;
    float threshold;
} Yin;

void Yin_init(Yin *yin, int16_t bufferSize, float threshold);
float Yin_getPitch(Yin *yin, int16_t* buffer);
float Yin_getProbability(Yin *yin);

#ifdef __cplusplus
}
#endif

#endif
