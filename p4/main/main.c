#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2s_tdm.h"
#include "driver/i2s_types.h"
#include "driver/i2s_common.h"

#include <string.h>

#define SAMPLE_RATE 44100
#define TDM_SLOTS 16
#define BITS_PER_SAMPLE 16
#define FRAMES_PER_CHUNK 256

#define I2S_MCLK_GPIO   GPIO_NUM_2
#define I2S_BCLK_GPIO   GPIO_NUM_3
#define I2S_WS_GPIO     GPIO_NUM_4
#define I2S_DOUT_GPIO   GPIO_NUM_5

typedef struct {
    uint32_t phase;
    uint32_t step;
} gen_state_t;

typedef int16_t sample_t;

static i2s_chan_handle_t tx_channel_handle = NULL;
static gen_state_t gen_state;

static void init_i2s_tdm(void);

static void i2s_tx_task(void* args);

void app_main(void)
{
    // void *buf = heap_caps_malloc(1024, MALLOC_CAP_DMA);
    // void *buf2 = heap_caps_malloc(1024, MALLOC_CAP_SPIRAM);
    // void *buf3 = heap_caps_malloc(1024, MALLOC_CAP_SIMD);
    // ESP_LOGI("DMA", "Buffer = %p", buf);
    // ESP_LOGI("SPIRAM", "Buffer = %p", buf2);
    // ESP_LOGI("SIMD", "Buffer = %p", buf3);
    // heap_caps_print_heap_info(MALLOC_CAP_DMA);
    // heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
    // heap_caps_print_heap_info(MALLOC_CAP_SIMD);

    // ESP_LOGI("MEM", "Freeing");
    // heap_caps_free(buf);
    // heap_caps_free(buf2);
    // heap_caps_free(buf3);
    // heap_caps_print_heap_info(MALLOC_CAP_DMA);
    // heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
    // heap_caps_print_heap_info(MALLOC_CAP_SIMD);

    init_i2s_tdm();

    xTaskCreatePinnedToCore(i2s_tx_task, "i2s_tx", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    
    for (;;)
    {
        ESP_LOGI("MAIN", "Running main task");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void init_i2s_tdm(void)
{
    ESP_LOGI("I2S_INIT", "Initializing I2S TDM16 channel");
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_frame_num = 128;
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_channel_handle, NULL));

    i2s_tdm_clk_config_t tdm_clk = {
        .sample_rate_hz     = SAMPLE_RATE,
        .clk_src            = I2S_CLK_SRC_APLL,
        .mclk_multiple      = I2S_MCLK_MULTIPLE_768,
        .bclk_div           = 0,
        .ext_clk_freq_hz    = 0,
    };

    const uint32_t all_16_slots_mask = (1U << TDM_SLOTS) - 1U;
    i2s_tdm_slot_config_t tdm_slot = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(BITS_PER_SAMPLE, I2S_SLOT_MODE_STEREO, all_16_slots_mask);
    tdm_slot.total_slot     = TDM_SLOTS;
    tdm_slot.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
    tdm_slot.ws_width       = I2S_TDM_AUTO_WS_WIDTH;
    tdm_slot.skip_mask      = false;

    i2s_tdm_gpio_config_t tdm_gpio = {
        .mclk = I2S_MCLK_GPIO,
        .bclk = I2S_BCLK_GPIO,
        .ws   = I2S_WS_GPIO,
        .dout = I2S_DOUT_GPIO,
        .din  = GPIO_NUM_NC,
        .invert_flags = {
            .mclk_inv = 0,
            .bclk_inv = 0,
            .ws_inv   = 0,
        },
    };

    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg  = tdm_clk,
        .slot_cfg = tdm_slot,
        .gpio_cfg = tdm_gpio,
    };

    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(tx_channel_handle, &tdm_cfg));

    gen_state.phase = 0;
    gen_state.step = 128;
    ESP_LOGI("I2S_INIT", "Initialized I2S TDM16 channel");
}


void i2s_tx_task(void* args)
{
    const char* TAG = "I2S_TX";
    const size_t frame_bytes = TDM_SLOTS * sizeof(sample_t);
    const size_t chunk_bytes = FRAMES_PER_CHUNK * frame_bytes;
    sample_t* chunk = (sample_t*)heap_caps_malloc(chunk_bytes, MALLOC_CAP_DEFAULT);
    ESP_ERROR_CHECK(chunk ? ESP_OK : ESP_ERR_NO_MEM);

    sample_t frame[TDM_SLOTS];

    ESP_ERROR_CHECK(i2s_channel_enable(tx_channel_handle));
    for (;;)
    {
        for (size_t f = 0; f < FRAMES_PER_CHUNK; ++f)
        {
            for (uint32_t s = 0; s < TDM_SLOTS; ++s)
            {
                frame[s] = (sample_t)((int32_t)(((gen_state.phase + (uint32_t)(s * 256)) & 0xFFFF) - 32768));
                memcpy(&chunk[f * (frame_bytes / sizeof(sample_t))], frame, frame_bytes);
                gen_state.phase += gen_state.step;
            }
        }

        size_t bytes_written = 0;
        esp_err_t ret = i2s_channel_write(tx_channel_handle, chunk, chunk_bytes, &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "i2s_channel_write failed: %s", esp_err_to_name(ret));
            // Yield briefly to avoid tight error loops
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        ESP_LOGI(TAG, "Wrote %d bytes", bytes_written);
        // vTaskDelay(pdMS_TO_TICKS(10));
    }

    heap_caps_free(chunk);
    vTaskDelete(NULL);
}