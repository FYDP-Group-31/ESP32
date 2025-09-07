#pragma once

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

class I2SAudioDevice {
    private:
        const char* TAG;
    public:
        void set_source(AudioSource_E source);

        virtual void init(void) = 0;
};

class ADAU1966A : public I2SAudioDevice {
    private:
        const char* TAG;

    public:
        ADAU1966A(const char* TAG);
        ~ADAU1966A();
};

class MAX98357 : public I2SAudioDevice {
    private:
        const char* TAG;

    public:
        MAX98357(const char* TAG);
        ~MAX98357();
};
