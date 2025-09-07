#include "i2s_audio.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2s_tdm.h"
#include "driver/i2s_std.h"
#include "driver/i2s_types.h"
#include "driver/i2s_common.h"

#include <string.h>
#include <math.h>
#include <memory>

#include "gpio_defs.h"

#define SAMPLE_RATE 48000
#define TDM_SLOTS 16
#define BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_16BIT

// Number of I2S frames per single write
#define FRAMES_PER_CHUNK 256

#define SLOT_MASK_TDM16 (i2s_tdm_slot_mask_t)((1U << TDM_SLOTS) - 1U)

// Type definitions
typedef int16_t sample_t;
typedef struct {
    uint32_t phase;
    uint32_t step;
} gen_state_t;

// static std::unique_ptr<ADAU1966A> adau1966a;
// static std::unique_ptr<MAX98357> max98357;

// Static function declarations
static bool _get_next_tdm_frame(sample_t* frame, uint8_t channels);

// Static variable declarations
static i2s_chan_handle_t i2s_audio_adau1966a_channel = NULL;
static i2s_chan_handle_t i2s_audio_max98357_channel = NULL;

static AudioSource_E audio_source = SOURCE_IDLE;
static gen_state_t gen_state;

void i2s_audio_adau1966a_init(void)
{
    const char* TAG = "ADAU1966A";

    // Initialize channel
    ESP_LOGI(TAG, "Initializing I2S Audio ADAU1966A peripheral");

    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 120,
        .auto_clear = true,
        .auto_clear_before_cb = false,
        .allow_pd = false,
        .intr_priority = 0
    };

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_audio_adau1966a_channel, NULL));

    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg  = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_APLL,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_768,
            .bclk_div = 0
        },
        .slot_cfg = {
            .data_bit_width = BITS_PER_SAMPLE,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = SLOT_MASK_TDM16,
            .ws_width = I2S_TDM_AUTO_WS_WIDTH,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
            .skip_mask = false,
            .total_slot = TDM_SLOTS
        },
        .gpio_cfg = {
            .mclk = I2S_MCLK_GPIO,
            .bclk = I2S_BCLK_GPIO,
            .ws   = I2S_WS_GPIO,
            .dout = I2S_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = 0,
                .bclk_inv = 0,
                .ws_inv   = 0,
            }
        }
    };

    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(i2s_audio_adau1966a_channel, &tdm_cfg));

    gen_state.phase = 0;
    gen_state.step = 128;
}

void i2s_audio_max98357_init(void)
{
    const char* TAG = "MAX98357";

    gpio_config_t en_io = {
        .pin_bit_mask = 1ULL << MAX98357_EN_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE
    };
    gpio_config(&en_io);
    gpio_set_level(MAX98357_EN_GPIO, MAX98357_ENABLE);

    ESP_LOGI(TAG, "Initializing I2S for MAX98357");
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_audio_max98357_channel, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            .bclk_div = 0
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,   // MAX98357 doesn’t need MCLK
            .bclk = MAX98357_BCLK_GPIO,
            .ws   = MAX98357_WS_GPIO,
            .dout = MAX98357_DATA_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_audio_max98357_channel, &std_cfg));
}

// TODO: Replace with more descriptive type than bool
bool i2s_audio_set_source(AudioSource_E new_source)
{
    bool ret = true;

    // TODO: Use RTOS queues + task for thread safety and sync
    audio_source = new_source;

    return ret;
}

static bool _get_next_tdm_frame(sample_t* frame, uint8_t channels)
{
    bool ret = true;

    switch (audio_source)
    {
        case SOURCE_INTERNAL:
        {
            for (uint8_t slot = 0; slot < channels; ++slot)
            {
                frame[slot] = (sample_t)((int32_t)(((gen_state.phase + (uint32_t)(slot * 256)) & 0xFFFF) - 32768));
                gen_state.phase += gen_state.step;
            }
            break;
        }
        case SOURCE_EXT_UART:
        case SOURCE_EXT_A2DP:
        case SOURCE_IDLE:
        default:
        {
            ret = false;
            break;
        }
    }

    return ret;
}

void i2s_audio_adau1966a_task(void* args)
{
    const char* TAG = "ADAU1966A";
    const size_t bytes_per_frame = TDM_SLOTS * sizeof(sample_t);
    const size_t bytes_per_chunk = FRAMES_PER_CHUNK * bytes_per_frame;
    sample_t* chunk = (sample_t*)heap_caps_malloc(bytes_per_chunk, MALLOC_CAP_DMA);
    memset(chunk, 0, bytes_per_chunk);
    sample_t tdm_frame[TDM_SLOTS] = {0};

    ESP_ERROR_CHECK(i2s_channel_enable(i2s_audio_adau1966a_channel));
    for (;;)
    {
        for (size_t frame = 0; frame < FRAMES_PER_CHUNK; ++frame)
        {
            if (_get_next_tdm_frame(tdm_frame, TDM_SLOTS) == true)
            {
                memcpy(&chunk[frame * (bytes_per_frame / sizeof(sample_t))], tdm_frame, bytes_per_frame);
            }
            else
            {

            }
        }

        size_t bytes_written = 0;
        esp_err_t ret = i2s_channel_write(i2s_audio_adau1966a_channel, chunk, bytes_per_chunk, &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s_channel_write failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else
        {
            ESP_LOGI(TAG, "Wrote %d bytes on core %d", bytes_written, xPortGetCoreID());
        }
    }
    heap_caps_free(chunk);
    vTaskDelete(NULL);
}

void i2s_audio_max98357_task(void* args)
{
    const char* TAG = "MAX98357";
    const size_t bytes_per_chunk = 512 * 2 * sizeof(sample_t);
    sample_t* chunk = (sample_t*)heap_caps_malloc(bytes_per_chunk, MALLOC_CAP_DMA);
    memset(chunk, 0, bytes_per_chunk);

    float phase = 0.0f;
    const float two_pi = 6.28318530718f;
    const float phase_inc = two_pi * 440 / (float)SAMPLE_RATE;
    const int16_t amp = 20000;
    
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_audio_max98357_channel));
    for (;;)
    {
        for (size_t n = 0; n < 512; ++n) {
            float s = sinf(phase);
            phase += phase_inc;
            if (phase >= two_pi) phase -= two_pi;

            int16_t sample = (int16_t)(amp * s);

            // Interleaved stereo: L,R,L,R,...
            chunk[2*n + 0] = sample;   // Left
            chunk[2*n + 1] = sample;   // Right
        }
        size_t bytes_written = 0;
        esp_err_t ret = i2s_channel_write(i2s_audio_max98357_channel, chunk, bytes_per_chunk, &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s_channel_write failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else
        {
            ESP_LOGI(TAG, "(MAX) Wrote %d bytes on core %d", bytes_written, xPortGetCoreID());
        }
    }
    heap_caps_free(chunk);
    vTaskDelete(NULL);
}

ADAU1966A::ADAU1966A(const char* TAG)
:   TAG(TAG)
{

}

MAX98357::MAX98357(const char* TAG)
:   TAG(TAG)
{

}
