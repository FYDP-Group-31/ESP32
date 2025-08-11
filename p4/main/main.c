#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

void app_main(void)
{
    void *buf = heap_caps_malloc(1024, MALLOC_CAP_DMA);
    void *buf2 = heap_caps_malloc(1024, MALLOC_CAP_SPIRAM);
    void *buf3 = heap_caps_malloc(1024, MALLOC_CAP_SIMD);
    ESP_LOGI("DMA", "Buffer = %p", buf);
    ESP_LOGI("SPIRAM", "Buffer = %p", buf2);
    ESP_LOGI("SIMD", "Buffer = %p", buf3);
    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
    heap_caps_print_heap_info(MALLOC_CAP_DMA);
    heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
    
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
