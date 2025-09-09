// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// C++
#include <cassert>

// ESP-IDF
#include "esp_log.h"

// Components
#include "adau1966a.hpp"
#include "max98357a.hpp"

extern "C" {
    void app_main(void);
}

void app_main(void)
{
    assert(init_adau1966a(I2S_MCLK_GPIO, I2S_BCLK_GPIO, I2S_WS_GPIO, I2S_DOUT_GPIO));
    ADAU1966A& adau1966a = get_adau1966a();
    adau1966a.start_thread();
}
