#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// #include "esp_a2dp_api.h"

// Custom components
#include "i2s_driver.hpp"

extern "C" {
    void app_main(void);
}

// GPIO definitions
#define LED_PIN             GPIO_NUM_2

#define DATA_IN_BUF_LEN 2048
#define DATA_OUT_BUF_LEN 2048

extern uint32_t runtime_ms;

uint8_t program_run;

uint8_t *data_in_buf;
uint8_t *data_out_buf;

QueueHandle_t data_queue;

static bool mem_init(void);
static void mem_deinit(void);

static void task_toggle_led(void *args)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    bool led_level = false;

    for (;;)
    {
        led_level = !led_level;
        gpio_set_level(LED_PIN, (uint32_t)led_level);
        printf("runtime = %ld\n", runtime_ms);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

static void task_data_read(void *args)
{
    assert(data_in_buf != NULL);
    assert(data_queue != 0);

    for (;;)
    {
        taskYIELD();
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
    
    if (i2s_driver_ADAU1966A.init() == false)
    {

    }

    program_run = 1;

    xTaskCreate(task_toggle_led, "toggle_led", 4096, NULL, 1, NULL);
    xTaskCreate(task_data_read, "data_read", 4096, NULL, 24, NULL);

    while (program_run == 1)
    {
        taskYIELD();
    }

    mem_deinit();

    vTaskDelete(NULL);
}