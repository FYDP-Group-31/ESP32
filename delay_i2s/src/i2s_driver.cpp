#include "i2s_driver.hpp"

#include "driver/i2s_std.h"

#define I2S_TX_BCLK_PIN     GPIO_NUM_4
#define I2S_TX_WS_PIN       GPIO_NUM_5
#define I2S_TX_DATA_PIN     GPIO_NUM_18

#define AUDIO_SAMPLE_RATE 44100
#define NUM_AUDIO_CHANNELS 16
#define ADAU1966A_BIT_WIDTH I2S_DATA_BIT_WIDTH_24BIT

I2S_Driver i2s_driver_ADAU1966A(
    NUM_AUDIO_CHANNELS,
    AUDIO_SAMPLE_RATE,
    ADAU1966A_BIT_WIDTH
);

static void i2s_write_task(void* args)
{
    for (;;)
    {
        taskYIELD();
    }
    vTaskDelete(NULL);
}

I2S_Driver::I2S_Driver(uint8_t num_audio_channels, uint32_t audio_sample_rate, i2s_data_bit_width_t bit_width)
:   tx_ch_handle(NULL),
    fifo_handle(NULL),
    num_audio_channels(num_audio_channels),
    audio_sample_rate(audio_sample_rate),
    bit_width(bit_width)
{

}

I2S_Driver::~I2S_Driver()
{

}

bool I2S_Driver::init()
{
    // Initialize data structures
    if ((this->fifo_handle = xQueueCreate(2048, sizeof(uint8_t))) == NULL)
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
            .bclk = I2S_TX_BCLK_PIN,
            .ws = I2S_TX_WS_PIN,
            .dout = I2S_TX_DATA_PIN,
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
    if (xTaskCreate(i2s_write_task, "i2s write", 4096, NULL, 24, NULL) != pdPASS)
    {
        return false;
    }

    return true;
}