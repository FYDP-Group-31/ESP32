#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// Custom components
#include "adau1966a_driver.hpp"
#include "bluetooth_receiver.hpp"
#include "gpio_defs.h"

extern "C" {
    void app_main(void);
}

#define DATA_IN_BUF_LEN 2048
#define DATA_OUT_BUF_LEN 2048

extern uint32_t runtime_ms;

uint8_t program_run;

uint8_t *data_in_buf;
uint8_t *data_out_buf;

static bool mem_init(void);
static void mem_deinit(void);

static void task_toggle_led(void *args)
{
    gpio_set_direction(GPIO_LED_PIN, GPIO_MODE_OUTPUT);

    bool led_level = false;

    for (;;)
    {
        led_level = !led_level;
        gpio_set_level(GPIO_LED_PIN, (uint32_t)led_level);
        printf("runtime = %ld\n", runtime_ms);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

static void task_data_read(void *args)
{
    assert(data_in_buf != NULL);

    for (;;)
    {
        // taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelete(NULL);
}

static bool mem_init(void)
{
    if ((data_in_buf = (uint8_t*)calloc(1, DATA_IN_BUF_LEN)) == NULL)
    {
        return false;
    }

    if ((data_out_buf = (uint8_t*)calloc(1, DATA_OUT_BUF_LEN)) == NULL)
    {
        return false;
    }


    return true;
}

static void mem_deinit(void)
{
    free(data_in_buf);
    free(data_out_buf);
}

void app_main(void)
{
    mem_init();
    

    // Commented out until A2DP can be fully implemented
    // if (bt_receiver.init() == false)
    // {
    //     printf("BT client init failed\n");
    // }

    if (adau1966a_driver.init() == false)
    {
        printf("ADAU1966A Init failed\n");
    }

    

    program_run = 1;

    xTaskCreate(task_toggle_led, "toggle_led", 4096, NULL, 1, NULL);
    xTaskCreate(task_data_read, "data_read", 4096, NULL, 24, NULL);

    adau1966a_driver.start_threads();

    while (program_run == 1)
    {
        taskYIELD();
    }

    mem_deinit();

    vTaskDelete(NULL);
}