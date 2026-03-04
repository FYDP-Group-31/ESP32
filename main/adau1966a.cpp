#include "adau1966a.hpp"

#include <memory>
#include <cassert>
#include <cmath>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_dsp.h"
#include "driver/gpio.h"

#define DEBUG 0

#define BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_16BIT

static inline constexpr size_t BYTES_PER_TDM_FRAME = (TDM_SLOTS * sizeof(sample_t));
static inline constexpr size_t FRAMES_PER_I2S_CHUNK = 256;
static inline constexpr size_t BYTES_PER_I2S_CHUNK = FRAMES_PER_I2S_CHUNK * BYTES_PER_TDM_FRAME;

// Stores 2 seconds of 16-bit mono (single-channel) audio data at 48kHz
static inline constexpr size_t audio_buf_size = sizeof(sample_t) * SAMPLE_RATE * 2;

static inline constexpr i2s_tdm_slot_mask_t SLOT_MASK_TDM16 = static_cast<i2s_tdm_slot_mask_t>((1U << TDM_SLOTS) - 1U);

static std::unique_ptr<ADAU1966A> _adau1966a;

bool init_adau1966a(gpio_num_t mclk_gpio, gpio_num_t bclk_gpio, gpio_num_t ws_gpio, gpio_num_t data_gpio)
{
  if (_adau1966a != NULL)
  {
    return true;
  }
  _adau1966a = std::make_unique<ADAU1966A>(mclk_gpio, bclk_gpio, ws_gpio, data_gpio);
  return _adau1966a->init();
}

ADAU1966A& get_adau1966a()
{
  assert(_adau1966a && "get_adau1966a() before init_adau1966a()");
  return *_adau1966a;
}


ADAU1966A::ADAU1966A(gpio_num_t mclk_gpio, gpio_num_t bclk_gpio, gpio_num_t ws_gpio, gpio_num_t data_gpio)
: mclk_gpio(mclk_gpio),
  bclk_gpio(bclk_gpio),
  ws_gpio(ws_gpio),
  data_gpio(data_gpio),
  i2s_driver(nullptr),
  bus_handle(nullptr),
  dev_handle(nullptr),
  sliding_window_buf(nullptr),
  sliding_window_read_idx(0),
  sliding_window_write_idx(0),
  sliding_window_remaining_size(0),
  channel_delay_offset{0},
  audio_mode(AUDIO_MODE_INPUT),
  chunk(nullptr),
  i2s_sliding_window(nullptr),
  thread_running(false),
  task(nullptr)
{

}

ADAU1966A::~ADAU1966A()
{
  this->deinit();
}

bool ADAU1966A::init()
{
  this->sliding_window_buf = (sample_t*)heap_caps_malloc(
    audio_buf_size,
    MALLOC_CAP_DMA | MALLOC_CAP_SIMD
  );
  this->sliding_window_remaining_size = audio_buf_size / sizeof(sample_t);

  this->chunk = (sample_t*)heap_caps_malloc(BYTES_PER_I2S_CHUNK, MALLOC_CAP_DMA);
  if (this->chunk == nullptr)
  {
    ESP_LOGE(ADAU1966A::TAG, "Could not allocate %d bytes of DMA memory", BYTES_PER_I2S_CHUNK);
    return false;
  }
  memset(this->chunk, 0, BYTES_PER_I2S_CHUNK);

  // Allocate I2S sliding window (3 I2S chunks worth of mono samples)
  constexpr size_t SW_SAMPLES = 3 * FRAMES_PER_I2S_CHUNK;
  this->i2s_sliding_window = (sample_t*)heap_caps_malloc(
    SW_SAMPLES * sizeof(sample_t),
    MALLOC_CAP_DMA | MALLOC_CAP_SIMD
  );
  if (this->i2s_sliding_window == nullptr)
  {
    ESP_LOGE(ADAU1966A::TAG, "Could not allocate %d bytes for I2S sliding window", SW_SAMPLES * sizeof(sample_t));
    return false;
  }
  memset(this->i2s_sliding_window, 0, SW_SAMPLES * sizeof(sample_t));

  gpio_config_t uart_full_signal_gpio_cfg = {
    .pin_bit_mask = (1ULL << AUDIO_BUF_FULL_GPIO),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
    .hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&uart_full_signal_gpio_cfg));

  gpio_config_t dac_rst_gpio_cfg = {
    .pin_bit_mask = (1ULL << DAC_RST_GPIO),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type = GPIO_INTR_DISABLE,
    .hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&dac_rst_gpio_cfg));

  i2c_master_bus_config_t dac_i2c_bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = DAC_I2C_SDA_GPIO,
    .scl_io_num = DAC_I2C_SCL_GPIO,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 0,
    .trans_queue_depth = 48,
    .flags = {
      .enable_internal_pullup = 0,
      .allow_pd = 0
    }
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&dac_i2c_bus_config, &this->bus_handle));

  i2c_device_config_t dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = DAC_I2C_ADDR,
    .scl_speed_hz = DAC_I2C_FREQ_HZ,
    .scl_wait_us = 0,
    .flags = {
      .disable_ack_check = 0
    }
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(this->bus_handle, &dev_config, &this->dev_handle));

  i2s_chan_config_t i2s_cfg = {
    .id = I2S_NUM_0, // Make argument??
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = 6,
    .dma_frame_num = 120,
    .auto_clear = true,
    .auto_clear_before_cb = false,
    .allow_pd = false,
    .intr_priority = 0
  };
  ESP_ERROR_CHECK(i2s_new_channel(&i2s_cfg, &this->i2s_driver, NULL));

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
      .ws_width = 1,
      .ws_pol = true,
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

  ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(this->i2s_driver, &tdm_cfg));

  return true;
}

void ADAU1966A::deinit()
{
  ESP_ERROR_CHECK(i2s_channel_disable(this->i2s_driver));
  ESP_ERROR_CHECK(i2s_del_channel(this->i2s_driver));
  heap_caps_free(this->chunk);
  this->chunk = nullptr;

  heap_caps_free(this->i2s_sliding_window);
  this->i2s_sliding_window = nullptr;

  heap_caps_free(this->sliding_window_buf);
  this->sliding_window_buf = nullptr;
}

bool ADAU1966A::start_thread()
{
  if (this->task != nullptr)
  {
    return true;
  }
  this->thread_running = true;
  BaseType_t ret = xTaskCreatePinnedToCore(&ADAU1966A::i2s_thread_create, ADAU1966A::TAG, 4096, this, 5, &this->task, tskNO_AFFINITY);
  if (ret == pdPASS)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void ADAU1966A::stop_thread()
{
  if (this->task == nullptr)
  {
    return;
  }
  this->thread_running = false;
  this->task = nullptr;
}

void ADAU1966A::setup_dac()
{
  ESP_LOGI(ADAU1966A::TAG, "Resetting ADAU1966A using DAC_RST pin");
  gpio_set_level(DAC_RST_GPIO, 0);
  vTaskDelay(pdMS_TO_TICKS(1000));
  gpio_set_level(DAC_RST_GPIO, 1);
  vTaskDelay(pdMS_TO_TICKS(1000));

  ESP_LOGI(ADAU1966A::TAG, "Configuring ADAU1966A registers at I2C address %x", DAC_I2C_ADDR);

  ESP_ERROR_CHECK(i2s_channel_enable(this->i2s_driver));
  ESP_LOGI(ADAU1966A::TAG, "I2S channel enabled");

  const uint8_t pll_clk_ctrl0_val = (
    PLL_CLK_CTRL0_PLLIN_DLRCLK | // [7:6]
    PLL_CLK_CTRL0_XTAL_SET_OFF | // [5:4]
    PLL_CLK_CTRL0_SOFT_RST_NORMAL |  // [3]
    // Bit [2:1] DNC when PLL set to DLRCLK
    PLL_CLK_CTRL0_PUP_POWER_UP // [0]
  );
  const uint8_t pll_clk_ctrl0_buf[] = {PLL_CLK_CTRL0_REG, pll_clk_ctrl0_val};
  ESP_LOGI(ADAU1966A::TAG, "Reg 0x%x: 0x%x", PLL_CLK_CTRL0_REG , pll_clk_ctrl0_val);
  ESP_ERROR_CHECK(i2c_master_transmit(this->dev_handle, pll_clk_ctrl0_buf, sizeof(pll_clk_ctrl0_buf), -1));
  vTaskDelay(pdMS_TO_TICKS(1));

  const uint8_t pll_clk_ctrl1_val = (
    PLL_CLK_CTRL1_LOWPWR_MODE_I2C | // [7:6]
    PLL_CLK_CTRL1_MCLKO_SEL_DISABLED | // [5:4]
    PLL_CLK_CTRL1_PLL_MUTE_NO_AUTOMUTE | // [3]
    // Bit 2 read only
    PLL_CLK_CTRL1_VREF_EN_ENABLED | // [1]
    PLL_CLK_CTRL1_CLK_SEL_PLL // [0]
  );
  const uint8_t pll_clk_ctrl1_buf[] = {PLL_CLK_CTRL1_REG, pll_clk_ctrl1_val};
  ESP_LOGI(ADAU1966A::TAG, "Reg 0x%x: 0x%x", PLL_CLK_CTRL1_REG, pll_clk_ctrl1_val);
  ESP_ERROR_CHECK(i2c_master_transmit(this->dev_handle, pll_clk_ctrl1_buf, sizeof(pll_clk_ctrl1_buf), -1));
  vTaskDelay(pdMS_TO_TICKS(1));

  const uint8_t pdn_thrmsens_ctrl1_val = (
    PDN_THRMSENS_CTRL1_THRM_RATE_4S_CONVERSION | // [7:6]
    PDN_THRMSENS_CTRL1_THRM_MODE_ONE_SHOT | // [5]
    PDN_THRMSENS_CTRL1_THRM_GO_RESET | // [4]
    // Bit 3 reserved
    PDN_THRMSENS_CTRL1_TS_PDN_SENSOR_ON | // [2]
    PDN_THRMSENS_CTRL1_PLL_PDN_NORMAL_OPERATION | // [1]
    PDN_THRMSENS_CTRL1_VREG_PDN_NORMAL_OPERATION // [0]
  );
  const uint8_t pdn_thrmsens_ctrl1_buf[] = {PDN_THRMSENS_CTRL1_REG, pdn_thrmsens_ctrl1_val};
  ESP_LOGI(ADAU1966A::TAG, "Reg 0x%x: 0x%x", PDN_THRMSENS_CTRL1_REG, pdn_thrmsens_ctrl1_val);
  ESP_ERROR_CHECK(i2c_master_transmit(this->dev_handle, pdn_thrmsens_ctrl1_buf, sizeof(pdn_thrmsens_ctrl1_buf), -1));
  vTaskDelay(pdMS_TO_TICKS(1));

  const uint8_t dac_ctrl1_val = (
    DAC_CTRL1_BCLK_GEN_NORMAL_OPERATION |
    DAC_CTRL1_LRCLK_MODE_PULSE_MODE |
    DAC_CTRL1_LRCLK_POL_NORMAL |
    DAC_CTRL1_SAI_MSB_MSB_FIRST |
    // Bit 3 Reserved
    DAC_CTRL1_BCLK_RATE_32_PER_FRAME |
    DAC_CTRL1_BCLK_EDGE_LATCH_RISING_EDGE |
    DAC_CTRL1_SAI_MS_SLAVE
  );
  const uint8_t dac_ctrl1_buf[] = {DAC_CTRL1_REG, dac_ctrl1_val};
  ESP_LOGI(ADAU1966A::TAG, "Reg 0x%x: 0x%x", DAC_CTRL1_REG, dac_ctrl1_val);
  ESP_ERROR_CHECK(i2c_master_transmit(this->dev_handle, dac_ctrl1_buf, sizeof(dac_ctrl1_buf), -1));
  vTaskDelay(pdMS_TO_TICKS(1));

  const uint8_t dac_ctrl2_val = (
    // Bits [7:5] Reserved
    DAC_CTRL2_BCLK_TDMC_16_CYCLES_PER_SLOT |
    DAC_CTRL2_DAC_POL_NONINVERTED |
    DAC_CTRL2_AUTO_MUTE_EN_DISABLED |
    DAC_CTRL2_DAC_OSR_256_FS |
    DAC_CTRL2_DE_EMP_EN_DISABLED
  );
  const uint8_t dac_ctrl2_buf[] = {DAC_CTRL2_REG, dac_ctrl2_val};
  ESP_LOGI(ADAU1966A::TAG, "Reg 0x%x: 0x%x", DAC_CTRL2_REG, dac_ctrl2_val);
  ESP_ERROR_CHECK(i2c_master_transmit(this->dev_handle, dac_ctrl2_buf, sizeof(dac_ctrl2_buf), -1));
  vTaskDelay(pdMS_TO_TICKS(1));

  const uint8_t dac_ctrl0_val = (
    // Bits [7:6] DNC when bits [5:3] != 000
    DAC_CTRL0_SAI_MODE_TDM16_SINGLE | // [5:3]
    DAC_CTRL0_FS_32_44_1_48_KHZ | // [2:1]
    DAC_CTRL0_MMUTE_NORMAL_OPERATION // [0]
  );
  const uint8_t dac_ctrl0_buf[] = {DAC_CTRL0_REG, dac_ctrl0_val};
  ESP_LOGI(ADAU1966A::TAG, "Reg 0x%x: 0x%x", DAC_CTRL0_REG, dac_ctrl0_val);
  ESP_ERROR_CHECK(i2c_master_transmit(this->dev_handle, dac_ctrl0_buf, sizeof(dac_ctrl0_buf), -1));
  vTaskDelay(pdMS_TO_TICKS(1));

  this->set_volume(40.0f);
  constexpr float channel_atten_db[TDM_SLOTS] = {
    9.375f, 8.625f, 7.125f, 4.875f, 3.0f, 1.5f, 0.375f, 0.0f,
    0.0f, 0.375f, 1.5f, 3.0f, 4.875f, 7.125f, 8.625f, 9.375f
  };
  for (uint8_t ch = 0; ch < TDM_SLOTS; ++ch)
  {
    this->set_channel_volume(ch, channel_atten_db[ch]);
  }
}

// Private functions
void ADAU1966A::i2s_thread_create(void* pv)
{
  static_cast<ADAU1966A*>(pv)->run_i2s_thread();
  vTaskDelete(nullptr);
}

void ADAU1966A::set_volume(float new_attenuation_db)
{
  uint8_t attenuation_reg_value = vol_db_to_reg(new_attenuation_db);
  const uint8_t vol_buf[] = {DACMSTR_VOL_REG, attenuation_reg_value};
  ESP_LOGI(ADAU1966A::TAG, "Reg 0x%x: 0x%x (-%f dB)", DACMSTR_VOL_REG, attenuation_reg_value, new_attenuation_db);
  ESP_ERROR_CHECK(i2c_master_transmit(this->dev_handle, vol_buf, sizeof(vol_buf), 0));
}

void ADAU1966A::run_i2s_thread()
{
  this->setup_dac();

  // Pre-fill the I2S sliding window with silence
  memset(this->i2s_sliding_window, 0, 3 * FRAMES_PER_I2S_CHUNK * sizeof(sample_t));

  // Try to pre-fill all 3 chunks from the ring buffer
  for (int i = 0; i < 3; ++i)
  {
    this->consume_ringbuf(FRAMES_PER_I2S_CHUNK, this->i2s_sliding_window + i * FRAMES_PER_I2S_CHUNK);
  }

  // Test pattern state
  int square_counter = 0;
  sample_t square_amplitude = 5000;
  const float SINE_FREQUENCY = 440.0f;
  const float SINE_AMPLITUDE = 25000.0f;
  const float TWO_PI = 2.0f * M_PI;
  size_t sample_counter = 0;

  // for (size_t channel = 0U; channel < TDM_SLOTS; ++channel)
  // {
  //   this->set_channel_integer_delay_offset(channel, (channel * 5) - 40);
  // }

  for (;;)
  {
    if (!this->thread_running)
    {
      break;
    }

    this->audio_mode = AUDIO_MODE_INPUT;

    if (this->audio_mode == AUDIO_MODE_INPUT)
    {
      // === Sliding window pipeline ===
      // Write the middle chunk (index 1) to I2S, applying per-channel delay offsets.
      // Each channel reads from: i2s_sliding_window[FRAMES_PER_I2S_CHUNK + frame - delay_offset]
      // Positive delay_offset = delayed (reads from past/chunk 0)
      // Negative delay_offset = advanced (reads from future/chunk 2)
      constexpr size_t SW_TOTAL = 3 * FRAMES_PER_I2S_CHUNK;

      for (size_t frame = 0; frame < FRAMES_PER_I2S_CHUNK; ++frame)
      {
        for (uint8_t ch = 0; ch < TDM_SLOTS; ++ch)
        {
          int32_t idx = (int32_t)(FRAMES_PER_I2S_CHUNK + frame) - this->channel_delay_offset[ch];

          // Clamp to valid sliding window range
          if (idx < 0) idx = 0;
          else if ((size_t)idx >= SW_TOTAL) idx = SW_TOTAL - 1;

          this->chunk[frame * TDM_SLOTS + ch] = this->i2s_sliding_window[idx];
        }
      }

      size_t bytes_written = 0;
      esp_err_t ret = i2s_channel_write(this->i2s_driver, this->chunk, BYTES_PER_I2S_CHUNK, &bytes_written, portMAX_DELAY);
      if (ret != ESP_OK)
      {
        ESP_LOGE(ADAU1966A::TAG, "I2S write failed. Expected %dB, wrote %dB", BYTES_PER_I2S_CHUNK, bytes_written);
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      // Slide the window forward by one chunk:
      // Move chunks [1,2] -> positions [0,1]
      memmove(this->i2s_sliding_window, this->i2s_sliding_window + FRAMES_PER_I2S_CHUNK, 2 * FRAMES_PER_I2S_CHUNK * sizeof(sample_t));

      // Zero the invalidated chunk (new position 2)
      memset(this->i2s_sliding_window + 2 * FRAMES_PER_I2S_CHUNK, 0, FRAMES_PER_I2S_CHUNK * sizeof(sample_t));

      // Fill the new chunk from the ring buffer
      this->consume_ringbuf(FRAMES_PER_I2S_CHUNK, this->i2s_sliding_window + 2 * FRAMES_PER_I2S_CHUNK);

      // Flow control: signal ready when ring buffer usage drops below 25%
      size_t buffer_capacity = audio_buf_size / sizeof(sample_t);
      size_t used = buffer_capacity - this->sliding_window_remaining_size;
      if (used < buffer_capacity / 4)
      {
        this->signal_ringbuf_ready();
      }
    }
    else
    {
      // === Test pattern generation ===
      for (size_t frame_num = 0U; frame_num < FRAMES_PER_I2S_CHUNK; ++frame_num)
      {
        if (square_counter >= 240) {
          square_amplitude = -square_amplitude;
          square_counter = 0;
        }
        square_counter++;

        float phase = TWO_PI * SINE_FREQUENCY * sample_counter / SAMPLE_RATE;
        sample_t sine_value = (sample_t)(SINE_AMPLITUDE * sinf(phase));
        sample_counter++;

        for (uint8_t channel_num = 0U; channel_num < TDM_SLOTS; ++channel_num)
        {
          switch (this->audio_mode)
          {
            case AUDIO_MODE_SINE_WAVE:
            {
              this->chunk[frame_num * TDM_SLOTS + channel_num] = sine_value;
              break;
            }
            case AUDIO_MODE_SQUARE_WAVE:
            {
              this->chunk[frame_num * TDM_SLOTS + channel_num] = square_amplitude;
              break;
            }
            case AUDIO_MODE_SAWTOOTH:
            {
              break;
            }
            case AUDIO_MODE_INPUT:
            default:
            {
              ESP_LOGE(ADAU1966A::TAG, "Invalid audio mode %d", this->audio_mode);
              break;
            }
          }
        }
      }

      size_t bytes_written = 0;

      esp_err_t ret = i2s_channel_write(this->i2s_driver, this->chunk, BYTES_PER_I2S_CHUNK, &bytes_written, portMAX_DELAY);

      if (ret != ESP_OK)
      {
        ESP_LOGE(ADAU1966A::TAG, "Failed to write chunk to I2S. %dB expected, %dB written", BYTES_PER_I2S_CHUNK, bytes_written);
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
  }
  ESP_ERROR_CHECK(i2s_channel_disable(this->i2s_driver));
}

bool ADAU1966A::write_to_ringbuf(const sample_t* data, size_t num_samples)
{
  if (this->sliding_window_buf == nullptr)
  {
    ESP_LOGE(ADAU1966A::TAG, "Sliding window buffer not initialized");
    return false;
  }

  if (this->sliding_window_remaining_size < num_samples)
  {
    ESP_LOGW(ADAU1966A::TAG, "Not enough space in sliding window buffer to write data. Remaining size: %d samples, data size: %d samples", this->sliding_window_remaining_size, num_samples);
    return false;
  }

  size_t buffer_capacity = audio_buf_size / sizeof(sample_t);

  if (this->sliding_window_write_idx + num_samples > buffer_capacity)
  {
    // Need to split: data wraps around to the beginning of the buffer
    size_t first_part_samples = buffer_capacity - this->sliding_window_write_idx;
    size_t second_part_samples = num_samples - first_part_samples;
    
    // Copy first part to end of buffer
    memcpy(this->sliding_window_buf + this->sliding_window_write_idx, data, first_part_samples * sizeof(sample_t));
    
    // Copy second part to start of buffer
    memcpy(this->sliding_window_buf, data + first_part_samples, second_part_samples * sizeof(sample_t));
    
    this->sliding_window_write_idx = second_part_samples;
  }
  else
  {
    // No wrap around, single copy
    memcpy(this->sliding_window_buf + this->sliding_window_write_idx, data, num_samples * sizeof(sample_t));
    
    this->sliding_window_write_idx += num_samples;
  }

  this->sliding_window_remaining_size -= num_samples;

  // Signal UART full when ring buffer is >75% full (free < 25%)
  if (this->get_ringbuf_free_size() < (audio_buf_size / sizeof(sample_t)) / 4)
  {
    this->signal_ringbuf_full();
  }

  return true;
}

size_t ADAU1966A::get_ringbuf_free_size()
{
  return this->sliding_window_remaining_size;
}

size_t ADAU1966A::get_ringbuf_used_size()
{
  size_t buffer_capacity = audio_buf_size / sizeof(sample_t);
  return buffer_capacity - this->sliding_window_remaining_size;
}

size_t ADAU1966A::consume_ringbuf(size_t num_samples, sample_t* out_buf)
{
  if (this->sliding_window_buf == nullptr)
  {
    ESP_LOGE(ADAU1966A::TAG, "Ring buffer not initialized");
    return 0;
  }

  size_t buffer_capacity = audio_buf_size / sizeof(sample_t);
  size_t available = buffer_capacity - this->sliding_window_remaining_size;

  if (num_samples > available)
  {
    num_samples = available; // Only consume what's available; rest stays zero
  }

  if (num_samples == 0)
  {
    return 0;
  }

  size_t read_idx = this->sliding_window_read_idx;

  if (read_idx + num_samples > buffer_capacity)
  {
    // Wrap around
    size_t first = buffer_capacity - read_idx;
    size_t second = num_samples - first;
    memcpy(out_buf, this->sliding_window_buf + read_idx, first * sizeof(sample_t));
    memcpy(out_buf + first, this->sliding_window_buf, second * sizeof(sample_t));
  }
  else
  {
    memcpy(out_buf, this->sliding_window_buf + read_idx, num_samples * sizeof(sample_t));
  }

  this->sliding_window_read_idx = (read_idx + num_samples) % buffer_capacity;
  this->sliding_window_remaining_size += num_samples;

  return num_samples;
}

void ADAU1966A::signal_ringbuf_full()
{
  gpio_set_level(AUDIO_BUF_FULL_GPIO, 0);
}

void ADAU1966A::signal_ringbuf_ready()
{
  gpio_set_level(AUDIO_BUF_FULL_GPIO, 1);
}

void ADAU1966A::set_integer_delay_offset(int16_t pos, uint16_t depth)
{
  constexpr uint32_t SPEED_OF_SOUND = 343U; // m/s
  constexpr float CM_TO_M = 0.01f;

  // Speaker positions along the array axis relative to center (cm)
  constexpr float array_pos_cm[TDM_SLOTS] = {
    -68.6f, -54.5f, -42.5f, -32.4f, -23.7f, -16.1f, -9.3f, -3.1f,
    3.1f, 9.3f, 16.1f, 23.7f, 32.4f, 42.5f, 54.5f, 68.6f
  };

  // Compute distance from each speaker to the focal point (pos, depth) in cm
  float distance_cm[TDM_SLOTS];
  float max_distance_cm = 0.0f;

  for (uint8_t ch = 0; ch < TDM_SLOTS; ++ch)
  {
    float dx = array_pos_cm[ch] - (float)pos;
    float dz = (float)depth;
    distance_cm[ch] = sqrtf(dx * dx + dz * dz);
    if (distance_cm[ch] > max_distance_cm)
    {
      max_distance_cm = distance_cm[ch];
    }
  }

  // Delay-and-sum beamforming: the farthest speaker gets 0 delay
  // Closer speakers are delayed so all wavefronts arrive at the focal point simultaneously.
  for (uint8_t ch = 0; ch < TDM_SLOTS; ++ch)
  {
    float delay_m = (max_distance_cm - distance_cm[ch]) * CM_TO_M;
    float delay_s = delay_m / (float)SPEED_OF_SOUND;
    int32_t delay_samples = (int32_t)roundf(delay_s * (float)SAMPLE_RATE);

    this->set_channel_integer_delay_offset(ch, delay_samples);
    ESP_LOGI(ADAU1966A::TAG, "Channel %d: dist=%.1fcm, delay=%d samples", ch, distance_cm[ch], delay_samples);
  }
}

// Sets integer number of samples to delay channel output
void ADAU1966A::set_channel_integer_delay_offset(uint8_t channel, int32_t offset)
{
  if (channel >= TDM_SLOTS) {
    ESP_LOGE(ADAU1966A::TAG, "Invalid channel number %d for delay offset", channel);
    return;
  }
  // Clamp to sliding window limits (max 1 chunk in either direction)
  constexpr int32_t MAX_OFFSET = (int32_t)FRAMES_PER_I2S_CHUNK;
  if (offset > MAX_OFFSET) {
    ESP_LOGW(ADAU1966A::TAG, "Clamping channel %d delay offset from %d to %d", channel, offset, MAX_OFFSET);
    offset = MAX_OFFSET;
  } else if (offset < -MAX_OFFSET) {
    ESP_LOGW(ADAU1966A::TAG, "Clamping channel %d delay offset from %d to %d", channel, offset, -MAX_OFFSET);
    offset = -MAX_OFFSET;
  }
  this->channel_delay_offset[channel] = offset;
}

void ADAU1966A::set_channel_volume(uint8_t channel, float attenuation_db)
{
  if (channel >= TDM_SLOTS) {
    ESP_LOGE(ADAU1966A::TAG, "Invalid channel number %d for volume control", channel);
    return;
  }

  uint8_t attenuation_reg_value = vol_db_to_reg(attenuation_db);
  const uint8_t vol_buf[] = {static_cast<uint8_t>(DAC01_VOL_REG + channel), attenuation_reg_value};
  ESP_LOGI(ADAU1966A::TAG, "Reg 0x%x: 0x%x (-%f dB)", DAC01_VOL_REG + channel, attenuation_reg_value, attenuation_db);
  ESP_ERROR_CHECK(i2c_master_transmit(this->dev_handle, vol_buf, sizeof(vol_buf), 0));
}
