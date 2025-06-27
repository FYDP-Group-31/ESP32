#include "adau1966a_driver.hpp"

#include "audio_dsp.h"
#include "gpio_defs.h"
#include "esp_check.h"

#define AUDIO_SAMPLE_RATE 44100
#define NUM_AUDIO_CHANNELS 16
#define ADAU1966A_BIT_WIDTH I2S_DATA_BIT_WIDTH_16BIT

// ADAU1966A 16-bit mode
ADAU1966A_Driver adau1966a_driver(
    NUM_AUDIO_CHANNELS,
    AUDIO_SAMPLE_RATE,
    ADAU1966A_BIT_WIDTH
);

static void i2s_write_task(void* args)
{
    uint32_t count = 0;
    vTaskSuspend(NULL); // Start task when start_threads() is called

    for (;;)
    {
        printf("i2s_write_task %ld\n", count++);
        // taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

ADAU1966A_Driver::ADAU1966A_Driver(uint8_t num_audio_channels, uint32_t audio_sample_rate, i2s_data_bit_width_t bit_width)
:   tx_ch_handle(NULL),
    ringbuf(NULL),
    num_audio_channels(num_audio_channels),
    audio_sample_rate(audio_sample_rate),
    bit_width(bit_width)
{

}

ADAU1966A_Driver::~ADAU1966A_Driver()
{

}

bool ADAU1966A_Driver::init(void)
{
    // Initialize data structures
    if ((this->ringbuf = xRingbufferCreate(2048, RINGBUF_TYPE_BYTEBUF)) == NULL)
    {
        return false;
    }

    // I2S Channel initialization
    i2s_chan_config_t tx_ch_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(&tx_ch_cfg, &this->tx_ch_handle, NULL));
    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((this->num_audio_channels * this->audio_sample_rate) >> 2),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(this->bit_width, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_I2S_TX_BCLK,
            .ws = GPIO_I2S_TX_WS_PIN,
            .dout = GPIO_I2S_TX_DATA_OUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(this->tx_ch_handle, &tx_std_cfg));

    // Thread initialization
    if (xTaskCreate(i2s_write_task, "i2s write", 4096, NULL, 24, &this->write_task_handle) != pdPASS)
    {
        return false;
    }

    return true;
}

void ADAU1966A_Driver::start_threads(void)
{
    vTaskResume(this->write_task_handle);
}
