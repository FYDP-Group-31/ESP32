#ifndef _I2S_DRIVER_HPP_
#define _I2S_DRIVER_HPP_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"
#include "esp_check.h"

class I2S_Driver {
    private:
        i2s_chan_handle_t tx_ch_handle;
        QueueHandle_t fifo_handle;

        uint8_t num_audio_channels;
        uint32_t audio_sample_rate;
        i2s_data_bit_width_t bit_width;
    public:
        I2S_Driver(uint8_t num_audio_channels, uint32_t audio_sampling_rate, i2s_data_bit_width_t bit_width);
        ~I2S_Driver();

        bool init();
};

extern I2S_Driver i2s_driver_ADAU1966A;

#endif // _I2S_DRIVER_HPP_