#include "adau1966a.hpp"

#include <memory>
#include <cassert>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_dsp.h"

#define DEBUG 0

#define BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_16BIT

inline constexpr size_t BYTES_PER_TDM_FRAME = (TDM_SLOTS * sizeof(sample_t));
inline constexpr size_t FRAMES_PER_I2S_CHUNK = 256;
inline constexpr size_t BYTES_PER_I2S_CHUNK = FRAMES_PER_I2S_CHUNK * BYTES_PER_TDM_FRAME;

inline constexpr i2s_tdm_slot_mask_t SLOT_MASK_TDM16 = static_cast<i2s_tdm_slot_mask_t>((1U << TDM_SLOTS) - 1U);

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
  async_dma_driver(nullptr),
  uart_ringbuf_driver(nullptr),
  channel_delay_offset(nullptr),
  filter_buf(nullptr),
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
  this->filter_buf = (sample_t*)heap_caps_malloc(sizeof(sample_t) * 256, MALLOC_CAP_DMA);

  this->channel_frame_buf = (sample_t**)heap_caps_malloc(TDM_SLOTS * sizeof(sample_t*), MALLOC_CAP_8BIT);
  for (uint8_t channel = 0U; channel < TDM_SLOTS; ++channel)
  {
    this->channel_frame_buf[channel] = (sample_t*)heap_caps_malloc(FRAMES_PER_I2S_CHUNK * sizeof(sample_t), MALLOC_CAP_DMA);
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

  async_memcpy_config_t async_dma_config = {
    .backlog = 16U,
    .sram_trans_align = 0U,
    .dma_burst_size = BYTES_PER_TDM_FRAME,
    .flags = 0U
  };
  ESP_ERROR_CHECK(esp_async_memcpy_install(&async_dma_config, &this->async_dma_driver));

  this->channel_delay_offset = (size_t*)heap_caps_malloc(TDM_SLOTS * sizeof(size_t), MALLOC_CAP_8BIT);
  if (this->channel_delay_offset == nullptr)
  {
    ESP_LOGE(ADAU1966A::TAG, "Could not allocate %d bytes of memory for channel offsets", TDM_SLOTS * sizeof(size_t));
    return false;
  }
  for (uint8_t channel = 0U; channel < TDM_SLOTS; ++channel)
  {
    this->channel_delay_offset[channel] = 0U;
  }

  const size_t audio_buf_size = sizeof(sample_t) * SAMPLE_RATE * 2; // Stores 2 seconds of 16-bit mono (single-channel) audio data at 48kHz
  this->uart_ringbuf_driver = xRingbufferCreate(audio_buf_size, RINGBUF_TYPE_BYTEBUF);
  if (this->uart_ringbuf_driver == NULL) {
    ESP_LOGE(ADAU1966A::TAG, "Failed to create ring buffer");
    return false;
  }

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
  heap_caps_free(this->channel_delay_offset);
  this->channel_delay_offset = nullptr;

  for (uint8_t channel = 0U; channel < TDM_SLOTS; ++channel)
  {
    heap_caps_free(this->channel_frame_buf[channel]);
  }
  heap_caps_free(this->channel_frame_buf);
  this->channel_frame_buf = nullptr;

  heap_caps_free(this->filter_buf);
  this->filter_buf = nullptr;

  ESP_ERROR_CHECK(esp_async_memcpy_uninstall(this->async_dma_driver));
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

    // TODO: Use async memcpy to write to chunk
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
  if (this->uart_ringbuf_driver == NULL) {
    ESP_LOGE(ADAU1966A::TAG, "Ring buffer not initialized");
    return false;
  }

  size_t bytes_to_write = num_samples * sizeof(sample_t);
  size_t bytes_written = xRingbufferSend(this->uart_ringbuf_driver, (void*)data, bytes_to_write, pdMS_TO_TICKS(100));
  if (bytes_written != bytes_to_write) {
    ESP_LOGW(ADAU1966A::TAG, "Failed to write all data to ring buffer. Expected %d bytes, wrote %d bytes", bytes_to_write, bytes_written);
    return false;
  }

  return true;
}

size_t ADAU1966A::get_ringbuf_free_size()
{
  if (this->uart_ringbuf_driver == nullptr) {
    ESP_LOGE(ADAU1966A::TAG, "Ring buffer not initialized. Returned 0 for size.");
    return 0;
  }

  return xRingbufferGetCurFreeSize(this->uart_ringbuf_driver);
}

void ADAU1966A::signal_ringbuf_full()
{
  // Signal to RPi that ring buffer is full or almost full
  // Use single wire GPIO or UART message
}

void ADAU1966A::signal_ringbuf_ready()
{
  // Signal to RPi that ring buffer has enough space to write more data
  // Use single wire GPIO or UART message
}

size_t ADAU1966A::read_ringbuf(size_t num_samples, sample_t* out_buf)
{
  if (this->uart_ringbuf_driver == nullptr) {
    ESP_LOGE(ADAU1966A::TAG, "Ring buffer not initialized");
    return 0;
  }

  size_t bytes_to_read = num_samples * sizeof(sample_t);
  size_t bytes_read = 0;
  void* item = xRingbufferReceiveUpTo(this->uart_ringbuf_driver, &bytes_read, pdMS_TO_TICKS(100), bytes_to_read);
  if (item == NULL || bytes_read == 0) {
    ESP_LOGW(ADAU1966A::TAG, "No data available in ring buffer to read");
    return 0;
  }

  memcpy(out_buf, item, bytes_read);
  vRingbufferReturnItem(this->uart_ringbuf_driver, item);

  return bytes_read / sizeof(sample_t); // Should be equivalent to dividing by 2 (16-bit samples)
}

// Sets integer number of samples to delay channel output
void ADAU1966A::set_channel_integer_delay_offset(uint8_t channel, size_t offset)
{
  if (channel >= TDM_SLOTS) {
    ESP_LOGE(ADAU1966A::TAG, "Invalid channel number %d for delay offset", channel);
    return;
  }
  this->channel_delay_offset[channel] = offset;
}
