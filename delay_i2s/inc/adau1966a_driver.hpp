#ifndef _ADAU1966A_DRIVER_HPP_
#define _ADAU1966A_DRIVER_HPP_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/i2s_std.h"


class ADAU1966A_Driver {
    private:
        i2s_chan_handle_t tx_ch_handle;
        RingbufHandle_t ringbuf;

        uint8_t num_audio_channels;
        uint32_t audio_sample_rate;
        i2s_data_bit_width_t bit_width;

        TaskHandle_t write_task_handle;
    public:
        ADAU1966A_Driver(uint8_t num_audio_channels, uint32_t audio_sampling_rate, i2s_data_bit_width_t bit_width);
        ~ADAU1966A_Driver();

        bool init(void);
        void start_threads(void);
};

extern ADAU1966A_Driver adau1966a_driver;

#endif // _ADAU1966A_DRIVER_HPP_