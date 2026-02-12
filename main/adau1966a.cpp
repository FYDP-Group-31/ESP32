#include "adau1966a.hpp"

#include <memory>
#include <cassert>

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
  sliding_window_buf(nullptr),
  sliding_window_read_idx(0),
  sliding_window_write_idx(0),
  sliding_window_channel_idx{0},
  sliding_window_remaining_size(0),
  channel_delay_offset{0},
  channel_frame_buf(nullptr),
  chunk(nullptr),
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

  this->channel_frame_buf = (sample_t**)heap_caps_malloc(TDM_SLOTS * sizeof(sample_t*), MALLOC_CAP_8BIT);
  for (uint8_t channel = 0U; channel < TDM_SLOTS; ++channel)
  {
    this->channel_frame_buf[channel] = (sample_t*)heap_caps_malloc(
      FRAMES_PER_I2S_CHUNK * sizeof(sample_t),
      MALLOC_CAP_DMA | MALLOC_CAP_SIMD
    );
    if (this->channel_frame_buf[channel] == nullptr)
    {
      ESP_LOGE(ADAU1966A::TAG, "Could not allocate %d bytes of DMA memory for channel %d", FRAMES_PER_I2S_CHUNK * sizeof(sample_t), channel);
      return false;
    }
    memset(this->channel_frame_buf[channel], 0, FRAMES_PER_I2S_CHUNK * sizeof(sample_t));
  }

  this->chunk = (sample_t*)heap_caps_malloc(BYTES_PER_I2S_CHUNK, MALLOC_CAP_DMA);
  if (this->chunk == nullptr)
  {
    ESP_LOGE(ADAU1966A::TAG, "Could not allocate %d bytes of DMA memory", BYTES_PER_I2S_CHUNK);
    return false;
  }
  memset(this->chunk, 0, BYTES_PER_I2S_CHUNK);

  gpio_config_t uart_full_signal_gpio_cfg = {
    .pin_bit_mask = (1ULL << (AUDIO_BUF_FULL_GPIO - 1)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
    .hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&uart_full_signal_gpio_cfg));

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

  ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(this->i2s_driver, &tdm_cfg));

  return true;
}

void ADAU1966A::deinit()
{
  ESP_ERROR_CHECK(i2s_channel_disable(this->i2s_driver));
  ESP_ERROR_CHECK(i2s_del_channel(this->i2s_driver));
  heap_caps_free(this->chunk);
  this->chunk = nullptr;

  for (uint8_t channel = 0U; channel < TDM_SLOTS; ++channel)
  {
    heap_caps_free(this->channel_frame_buf[channel]);
  }
  heap_caps_free(this->channel_frame_buf);
  this->channel_frame_buf = nullptr;

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
  BaseType_t ret = xTaskCreatePinnedToCore(&ADAU1966A::thread_entry, ADAU1966A::TAG, 4096, this, 5, &this->task, tskNO_AFFINITY);
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


// Private functions
void ADAU1966A::thread_entry(void* pv)
{
  static_cast<ADAU1966A*>(pv)->run_thread();
  vTaskDelete(nullptr);
}

void ADAU1966A::run_thread()
{
  sample_t channel_frame[TDM_SLOTS] = {0};

  ESP_ERROR_CHECK(i2s_channel_enable(this->i2s_driver));
  for (;;)
  {
    if (this->thread_running == false)
    {
      break;
    }

    // TODO: Use read_ringbuf() for each channel to read required samples for delay filter, apply delay filter, and store result in channel_frame[channel_num]
    for (size_t frame_num = 0U; frame_num < FRAMES_PER_I2S_CHUNK; ++frame_num)
    {
      for (uint8_t channel_num = 0U; channel_num < TDM_SLOTS; ++channel_num)
      {
        this->chunk[frame_num * TDM_SLOTS + channel_num] = channel_frame[channel_num];
      }
    }
    size_t bytes_written = 0;
#if DEBUG
    int64_t start = esp_timer_get_time();
#endif // DEBUG
    esp_err_t ret = i2s_channel_write(this->i2s_driver, this->chunk, BYTES_PER_I2S_CHUNK, &bytes_written, portMAX_DELAY);
#if DEBUG
    int64_t end   = esp_timer_get_time();
#endif // DEBUG
    if (ret == ESP_OK)
    {
#if DEBUG
      ESP_LOGI(ADAU1966A::TAG, "Wrote %d bytes in %dus on core %d", bytes_written, (end - start), xPortGetCoreID());
#endif // DEBUG
    }
    else
    {
      ESP_LOGE(ADAU1966A::TAG, "Failed to write chunk to I2S channel. %dB expected, %dB written", BYTES_PER_I2S_CHUNK, bytes_written);
      vTaskDelay(pdMS_TO_TICKS(10));
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
    memcpy(
      this->sliding_window_buf + this->sliding_window_write_idx, 
      data, 
      first_part_samples * sizeof(sample_t)
    );
    
    // Copy second part to start of buffer
    memcpy(
      this->sliding_window_buf, 
      data + first_part_samples, 
      second_part_samples * sizeof(sample_t)
    );
    
    this->sliding_window_write_idx = second_part_samples;
  }
  else
  {
    // No wrap around, single copy
    memcpy(
      this->sliding_window_buf + this->sliding_window_write_idx, 
      data, 
      num_samples * sizeof(sample_t)
    );
    
    this->sliding_window_write_idx += num_samples;
  }

  this->sliding_window_remaining_size -= num_samples;

  // If remaining size is less than 25% of buffer capacity, signal to RPi to stop writing more data
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

void ADAU1966A::signal_ringbuf_full()
{
  gpio_set_level(AUDIO_BUF_FULL_GPIO, 0);
}

void ADAU1966A::signal_ringbuf_ready()
{
  gpio_set_level(AUDIO_BUF_FULL_GPIO, 1);
}

size_t ADAU1966A::read_ringbuf(size_t num_samples, int32_t offset, sample_t* out_buf)
{
  if (this->sliding_window_buf == nullptr) {
    ESP_LOGE(ADAU1966A::TAG, "Sliding window buffer not initialized");
    return 0;
  }

  size_t buffer_capacity = audio_buf_size / sizeof(sample_t);
  size_t samples_written = buffer_capacity - this->sliding_window_remaining_size;

  // Check if offset + num_samples exceeds available data
  if (offset + num_samples > samples_written) {
    ESP_LOGW(ADAU1966A::TAG, "Not enough data in buffer. Requested: %d samples at offset %d, Available: %d", num_samples, offset, samples_written);
    if (offset >= samples_written) {
      return 0; // Offset is beyond available data
    }
    num_samples = samples_written - offset; // Read what's available from offset
  }

  if (num_samples == 0) {
    ESP_LOGE(ADAU1966A::TAG, "Read of size 0 requested from ring buffer");
    return 0;
  }

  // Calculate read start index (offset from sliding_window_read_idx)
  size_t read_start_idx = (this->sliding_window_read_idx + offset + buffer_capacity) % buffer_capacity;

  if (read_start_idx + num_samples > buffer_capacity) {
    // Read wraps around the buffer boundary
    size_t first_part_samples = buffer_capacity - read_start_idx;
    size_t second_part_samples = num_samples - first_part_samples;

    // Copy first part from end of buffer
    memcpy(
      out_buf, 
      this->sliding_window_buf + read_start_idx, 
      first_part_samples * sizeof(sample_t)
    );

    // Copy second part from start of buffer
    memcpy(
      out_buf + first_part_samples, 
      this->sliding_window_buf, 
      second_part_samples * sizeof(sample_t)
    );
  } else {
    // Single contiguous read
    memcpy(
      out_buf, 
      this->sliding_window_buf + read_start_idx, 
      num_samples * sizeof(sample_t)
    );
  }

  return num_samples;
}

// Sets integer number of samples to delay channel output
void ADAU1966A::set_channel_integer_delay_offset(uint8_t channel, int32_t offset)
{
  if (channel >= TDM_SLOTS) {
    ESP_LOGE(ADAU1966A::TAG, "Invalid channel number %d for delay offset", channel);
    return;
  }
  this->channel_delay_offset[channel] = offset;
}
