#include "adau1966a_driver.hpp"

#include "audio_dsp.h"
#include "gpio_defs.h"
#include "esp_check.h"

#define AUDIO_SAMPLE_RATE 44100
#define NUM_AUDIO_CHANNELS 16
#define ADAU1966A_BIT_WIDTH I2S_DATA_BIT_WIDTH_16BIT

#define SINGLE_TONE_A4_FREQ 440

extern "C"
{
    
};

// Euclidean GCD algorithm
static uint16_t gcd(uint16_t a, uint16_t b)
{
    while (b != 0)
    {
        a %= b;
        a ^= b;
        b ^= a;
        a ^= b;
    }
    return a;
}

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
        switch (adau1966a_driver.getAudioSource())
        {
            case SOURCE_INTERNAL_TONE_GENERATOR:
            {
                printf("internal i2s_write_task %ld\n", count++);
                break;
            }
            case SOURCE_UART:
            {
                printf(" i2s_write_task %ld\n", count++);
                break;
            }
            case SOURCE_BLUETOOTH:
            {
                break;
            }
        }
        printf("i2s_write_task %ld\n", count++);
        // taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

static void internal_tone_generator_task(void* args)
{
    for (;;)
    {
        if (adau1966a_driver.getAudioSource() != SOURCE_INTERNAL_TONE_GENERATOR)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelete(NULL);
}

ADAU1966A_Driver::ADAU1966A_Driver(uint8_t num_audio_channels, uint32_t audio_sample_rate, i2s_data_bit_width_t bit_width)
:   tx_ch_handle(NULL),
    ringbuf(NULL),
    audio_source(SOURCE_INTERNAL_TONE_GENERATOR),
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


AudioSource ADAU1966A_Driver::getAudioSource(void)
{
    return this->audio_source;
}

void ADAU1966A_Driver::setAudioSource(AudioSource new_source)
{
    if (new_source == this->audio_source)
    {
        return;
    }

    // TODO: Mutex
    switch (new_source)
    {
        case SOURCE_INTERNAL_TONE_GENERATOR:
        {
            // xTaskCreate()
            break;
        }
        case SOURCE_UART:
        {
            break;
        }
        case SOURCE_BLUETOOTH:
        {
            break;
        }
        default:
        {
            break;
        }
    }
    this->audio_source = new_source;
}

TaskRet_t ADAU1966A_Driver::writeToBuffer(void)
{
    TaskRet_t ret = TASK_RET_OK;
    switch (this->audio_source)
    {
        case SOURCE_INTERNAL_TONE_GENERATOR:
        {
            break;
        }
        case SOURCE_UART:
        case SOURCE_BLUETOOTH:
        default:
        {
            // Data is provided externally
            ret = TASK_RET_WAIT;
            break;
        }
    }
    return ret;
}

// Private methods
bool ADAU1966A_Driver::generate_square_wave(uint16_t amp, uint16_t freq, uint16_t sample_rate, int16_t* buf, uint16_t buf_len, uint16_t* write_len)
{
    // Worst case: max_buf_size = sample_rate (sample_rate and freq have no GCD other than 1)
    /**
     * Example: freq = 440Hz (A4), sample rate = 44.1kHz
     * GCD(44100, 440) = 20
     * min_buf_size = (44100/20) = 2205
     * 2205 samples are needed to create a perfect loop
     */

    uint16_t min_buf_size = static_cast<uint16_t>(sample_rate / gcd(sample_rate, freq));
    if (freq > 20000)
    {
        return false;
    }
    else if (buf_len < min_buf_size)
    {
        return false;
    }
    for (uint16_t i = 0; i < min_buf_size; ++i)
    {
        // square_wave[i] = (i % (SAMPLE_RATE / TONE_FREQ_HZ) < (SAMPLE_RATE / TONE_FREQ_HZ) / 2) ? AMPLITUDE : -AMPLITUDE;
        if ((i % min_buf_size) < (min_buf_size >> 2))
        {
            buf[i] = amp;
        }
        else
        {
            buf[i] = -amp;
        }
    }
    *write_len = min_buf_size;
    return true;
}
