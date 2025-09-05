#ifndef _I2S_AUDIO_HPP_
#define _I2S_AUDIO_HPP_

#include <stdbool.h>

typedef enum {
    SOURCE_IDLE,
    SOURCE_EXT_UART,
    SOURCE_EXT_A2DP,
    SOURCE_INTERNAL
} AudioSource_E;

void i2s_audio_adau1966a_init(void);
void i2s_audio_max98357_init(void);

bool i2s_audio_set_source(AudioSource_E new_source);

void i2s_audio_adau1966a_task(void* args);
void i2s_audio_max98357_task(void* args);

#endif // _I2S_AUDIO_HPP_
