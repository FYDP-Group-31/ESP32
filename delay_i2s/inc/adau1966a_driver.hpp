#ifndef _ADAU1966A_DRIVER_HPP_
#define _ADAU1966A_DRIVER_HPP_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/i2s_std.h"

typedef enum {
    SOURCE_INTERNAL_TONE_GENERATOR,
    SOURCE_UART,
    SOURCE_BLUETOOTH
} AudioSource;

typedef enum {
    TASK_RET_OK,
    TASK_RET_WAIT,
    TASK_RET_FAIL
} TaskRet_t;

class ADAU1966A_Driver {
    private:
        i2s_chan_handle_t tx_ch_handle;
        RingbufHandle_t ringbuf;
        AudioSource audio_source;

        uint8_t num_audio_channels;
        uint32_t audio_sample_rate;
        i2s_data_bit_width_t bit_width;

        TaskHandle_t write_task_handle;

        bool generate_square_wave(uint16_t amp, uint16_t freq, uint16_t sample_rate, int16_t* buf, uint16_t buf_len, uint16_t* write_len);
    public:
        ADAU1966A_Driver(uint8_t num_audio_channels, uint32_t audio_sampling_rate, i2s_data_bit_width_t bit_width);
        ~ADAU1966A_Driver();

        bool init(void);
        void start_threads(void);

        AudioSource getAudioSource(void);
        void setAudioSource(AudioSource new_source);

        TaskRet_t writeToBuffer(void);
};

extern ADAU1966A_Driver adau1966a_driver;

#endif // _ADAU1966A_DRIVER_HPP_