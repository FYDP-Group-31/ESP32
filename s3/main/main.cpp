#include <stdio.h>
#include <stdint.h>
#include "led_strip.h"
#include "driver/gpio.h"
#include "driver/i2s_tdm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_it.h"
#include "s3_gpio_config.h"

#define DEBUG_MODE 0

extern "C" {
void app_main(void);
}

static i2s_chan_handle_t i2s_chan;

static void i2s_tdm_init(void);

static void generate_square_wave_task(void* args);
static void rgb_led_task(void* args);

void app_main(void)
{
    i2s_tdm_init();

    xTaskCreate(generate_square_wave_task, "Square wave generator", 2048, NULL, 5, NULL);
    xTaskCreate(rgb_led_task, "RGB LED", 4096, NULL, 10, NULL);

    for (;;)
    {
#if DEBUG_MODE
        printf("%lu\n", runtime_ms);
#endif
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void i2s_tdm_init(void)
{
    i2s_chan_config_t i2s_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&i2s_chan_cfg, &i2s_chan, NULL));

    i2s_tdm_slot_mask_t slot_mask = (i2s_tdm_slot_mask_t)(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3 | I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | I2S_TDM_SLOT6 | I2S_TDM_SLOT7);
    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_MONO,
            slot_mask
        ),
        .gpio_cfg = {
            .mclk = I2S_MCLK_GPIO,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = I2S_DATA_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(i2s_chan, &tdm_cfg));
}

static void generate_square_wave_task(void* args)
{
    // TODO: calculate required delay based on buffer size and sample rate
    // Whatever the time is, delay slightly less to allow for DMA transfer to complete
    // Use blocking calls: i2s_channel_write(handle, data, len, &bytes_written, portMAX_DELAY);

    i2s_chan_handle_t chan_handle;
    for (;;)
    {
        // i2s_channel_write();
#if DEBUG_MODE
        printf("test\n");
#endif
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
}

static void rgb_led_task(void* args)
{
    led_strip_handle_t led_strip;

    led_strip_config_t strip_config = {
        .strip_gpio_num = 48, // TODO: Add to GPIO defs
        .max_leds = 1,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // GRB sets mode to RGB and RGB sets mode to GRB??
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // RMT counter clock frequency
        .flags = {
            .with_dma = true,     // Using DMA can improve performance when driving more LEDs
        }
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    led_strip_clear(led_strip);

    uint8_t brightness = 0;
    for (;;)
    {
#if DEBUG_MODE
        printf("Brightness %d\n", brightness);
#endif
        led_strip_set_pixel(led_strip, 0, brightness, 0, 0);
        led_strip_refresh(led_strip);
        if(++brightness > 50)
        {
            brightness = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    vTaskDelete(NULL);
}
