#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_tdm.h"
#include "driver/i2s_types.h"
#include "driver/i2s_common.h"

#include "gpio_defs.h"

// Type macros
typedef int16_t sample_t;

class ADAU1966A {
    private:
        gpio_num_t mclk_gpio;
        gpio_num_t bclk_gpio;
        gpio_num_t ws_gpio;
        gpio_num_t data_gpio;

        i2s_chan_handle_t channel;

        sample_t* chunk;

        bool thread_running;
        TaskHandle_t task;

        static void thread_entry(void* pv);
        void run_thread();

    public:
        static constexpr const char* TAG = "ADAU1966A";

        explicit ADAU1966A(
            gpio_num_t mclk_gpio,
            gpio_num_t bclk_gpio,
            gpio_num_t ws_gpio,
            gpio_num_t data_gpio
        );
        ~ADAU1966A();

        bool init();
        void deinit();
        bool start_thread();
        void stop_thread();
};

bool init_adau1966a(gpio_num_t mclk_gpio, gpio_num_t bclk_gpio, gpio_num_t ws_gpio, gpio_num_t data_gpio);
ADAU1966A& get_adau1966a();

