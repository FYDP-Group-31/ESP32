#include <stdio.h>
#include <stdint.h>
#include "led_strip.h"

#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/i2s_tdm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_it.h"
#include "s3_gpio_config.h"

#define DEBUG_MODE 1
#define DEBUG_TIMER 1
#define DEBUG_LED 0
#define DEBUG_I2S 1

#define NUM_TDM_CHANNELS 8
#define AUDIO_BYTE_WIDTH 2

extern "C" {
void app_main(void);
}

static i2s_chan_handle_t i2s_chan;
static int16_t* i2s_tdm_buf;

static void i2s_tdm_init(void);
static void i2s_tdm_deinit(void);

static void generate_square_wave_task(void* args);
static void rgb_led_task(void* args);

void app_main(void)
{
    i2s_tdm_init();

    xTaskCreate(generate_square_wave_task, "Square wave generator", 8192, NULL, 5, NULL);
    xTaskCreate(rgb_led_task, "RGB LED", 4096, NULL, 10, NULL);

    for (;;)
    {
#if DEBUG_MODE && DEBUG_TIMER
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
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_chan));

#if DEBUG_MODE && DEBUG_I2S
    printf("I2S channel initialized\n");
#endif
    
}

static void i2s_tdm_deinit(void)
{
    
}

static void generate_square_wave_task(void* args)
{
    // TODO: calculate required delay based on buffer size and sample rate
    // Whatever the time is, delay slightly less to allow for DMA transfer to complete
    // Use blocking calls: i2s_channel_write(handle, data, len, &bytes_written, portMAX_DELAY);

#if DEBUG_MODE && DEBUG_I2S
    printf("Square wave generator task begin");
#endif

    size_t frame_size = 256;
    size_t buf_len = NUM_TDM_CHANNELS * AUDIO_BYTE_WIDTH * frame_size;
    i2s_tdm_buf = (int16_t*)heap_caps_malloc(buf_len, MALLOC_CAP_DMA);
#if DEBUG_MODE && DEBUG_I2S
    printf("Write buffer allocated at %p\n", (void*)i2s_tdm_buf);
#endif
    assert(i2s_tdm_buf != NULL);

    int16_t amplitude[NUM_TDM_CHANNELS] = {0};
    uint16_t phase[NUM_TDM_CHANNELS] = {0};
    float period[NUM_TDM_CHANNELS] = {0.0f};
    float frequency;

    for (uint8_t channel = 0; channel < NUM_TDM_CHANNELS; ++channel)
    {
        frequency = 220.0 * (channel + 1);
        amplitude[channel] = 10000;
        phase[channel] = 0;
        period[channel] = 44100.0 / (frequency * 2);
    }

    size_t bytes_written;
    size_t index;
    esp_err_t write_status;
    for (;;)
    {
        for (size_t _frame = 0; _frame < frame_size; ++_frame)
        {
            for (uint8_t _ch = 0; _ch < NUM_TDM_CHANNELS; ++_ch)
            {
                index = (_frame * NUM_TDM_CHANNELS) + _ch;
                i2s_tdm_buf[index] = amplitude[_ch];

                ++phase[_ch];
                if (phase[_ch] >= period[_ch])
                {
                    phase[_ch] = 0;
                    amplitude[_ch] = -amplitude[_ch];
                }
            }
        }

        write_status = i2s_channel_write(i2s_chan, i2s_tdm_buf, buf_len, &bytes_written, portMAX_DELAY);
        switch (write_status)
        {
            case ESP_OK:
            {
#if DEBUG_MODE && DEBUG_I2S
                printf("%u bytes written\n", bytes_written);
#endif
                break;
            }
            case ESP_ERR_INVALID_ARG:
            {
#if DEBUG_MODE && DEBUG_I2S
                printf("Invalid I2S handle or NULL pointer\n");
#endif
                break;
            }
            case ESP_ERR_TIMEOUT:
            {
#if DEBUG_MODE && DEBUG_I2S
                printf("I2S write timeout\n");
#endif
                break;
            }
            case ESP_ERR_INVALID_STATE:
            {
#if DEBUG_MODE && DEBUG_I2S
                printf("I2S channel not enabled\n");
#endif
                break;
            }
            default:
            {
#if DEBUG_MODE && DEBUG_I2S
                printf("Unknown return to i2s_channel_write: %d\n", write_status);
#endif
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    heap_caps_free(i2s_tdm_buf);
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
#if DEBUG_MODE && DEBUG_LED
        printf("Brightness %d\n", brightness);
#endif
        led_strip_set_pixel(led_strip, 0, brightness, 0, 0);
        led_strip_refresh(led_strip);
        if (brightness)
        {
            brightness = 0;
        }
        else
        {
            brightness = 16;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}
