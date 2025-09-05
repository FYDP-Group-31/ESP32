#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "i2s_audio.hpp"

extern "C" {
    void app_main(void);
}

void app_main(void)
{   
    i2s_audio_adau1966a_init();
    i2s_audio_max98357_init();
    
    xTaskCreatePinnedToCore(i2s_audio_adau1966a_task, "ADAU1966A", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(i2s_audio_max98357_task, "MAX98357", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    
    for (;;)
    {
        ESP_LOGI("IDLE", "Running idle task");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
