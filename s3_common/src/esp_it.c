#include "esp_it.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include <stdio.h>

volatile uint32_t runtime_ms = 0;

void IRAM_ATTR vApplicationTickHook(void)
{
    switch (xPortGetCoreID())
    {
        case 0: // System Core
        {
            break;
        }
        case 1: // App core
        {
            ++runtime_ms;
            break;
        }
        default:
        {
            break;
        }
    }
}
