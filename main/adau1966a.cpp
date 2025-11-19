#include "adau1966a.hpp"

#include <memory>
#include <cassert>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_dsp.h"

#define DEBUG 0

#define SAMPLE_RATE 48000
#define TDM_SLOTS 16
#define BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_16BIT

#define FRAMES_PER_CHUNK 256

#define SLOT_MASK_TDM16 (static_cast<i2s_tdm_slot_mask_t>((1U << TDM_SLOTS) - 1U))

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
  channel(nullptr),
  async_dma_driver(nullptr),
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
  const size_t bytes_per_frame = TDM_SLOTS * sizeof(sample_t);
  const size_t bytes_per_chunk = FRAMES_PER_CHUNK * bytes_per_frame;
  this->chunk = (sample_t*)heap_caps_malloc(bytes_per_chunk, MALLOC_CAP_DMA);
  if (this->chunk == nullptr)
  {
    ESP_LOGE(ADAU1966A::TAG, "Could not allocate %d bytes of DMA memory", bytes_per_chunk);
    return false;
  }
  memset(this->chunk, 0, bytes_per_chunk);

  async_memcpy_config_t async_dma_config = {
    .backlog = 16U,
    .sram_trans_align = 0U,
    .dma_burst_size = bytes_per_frame,
    .flags = 0U
  };
  ESP_ERROR_CHECK(esp_async_memcpy_install(&async_dma_config, &this->async_dma_driver));

  i2s_chan_config_t channel_cfg = {
    .id = I2S_NUM_0, // Make argument??
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = 6,
    .dma_frame_num = 120,
    .auto_clear = true,
    .auto_clear_before_cb = false,
    .allow_pd = false,
    .intr_priority = 0
  };
  ESP_ERROR_CHECK(i2s_new_channel(&channel_cfg, &this->channel, NULL));

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

  ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(this->channel, &tdm_cfg));

  return true;
}

void ADAU1966A::deinit()
{
  ESP_ERROR_CHECK(i2s_channel_disable(this->channel));
  ESP_ERROR_CHECK(i2s_del_channel(this->channel));
  heap_caps_free(this->chunk);
  this->chunk = nullptr;

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
  const size_t bytes_per_frame = TDM_SLOTS * sizeof(sample_t);
  const size_t bytes_per_chunk = FRAMES_PER_CHUNK * bytes_per_frame;
  sample_t channel_frame[TDM_SLOTS] = {0};

  ESP_ERROR_CHECK(i2s_channel_enable(this->channel));
  for (;;)
  {
    if (this->thread_running == false)
    {
      break;
    }

    for (size_t frame_num = 0U; frame_num < FRAMES_PER_CHUNK; ++frame_num)
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
    esp_err_t ret = i2s_channel_write(this->channel, this->chunk, bytes_per_chunk, &bytes_written, portMAX_DELAY);
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
      ESP_LOGE(ADAU1966A::TAG, "Failed to write chunk to I2S channel. %dB expected, %dB written", bytes_per_chunk, bytes_written);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
      
  }
  ESP_ERROR_CHECK(i2s_channel_disable(this->channel));
}