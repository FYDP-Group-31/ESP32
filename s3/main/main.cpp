#include <stdio.h>
#include <stdint.h>
// #include "esp_it.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
void app_main(void);
}

extern volatile uint32_t runtime_ms;

void app_main(void)
{
    for (;;)
    {
        printf("%lu\n", runtime_ms);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}