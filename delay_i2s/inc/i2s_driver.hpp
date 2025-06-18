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
    public:
        I2S_Driver();
        ~I2S_Driver();

        bool init();
};

#endif // _I2S_DRIVER_HPP_