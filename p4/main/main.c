#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

void app_main(void)
{
    void *buf = heap_caps_malloc(2 * 1024 * 1024, MALLOC_CAP_SPIRAM);
    ESP_LOGI("PSRAM", "Buffer = %p", buf);
    assert(buf != NULL);
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
