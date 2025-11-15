#include "uart_comm.hpp"

#include <memory>
#include <cassert>

#include "esp_log.h"
#include "esp_heap_caps.h"

typedef enum {
  // STATE_READ_IDLE,
  STATE_READ_ADDR,
  STATE_READ_CMD,
  STATE_READ_LEN,
  STATE_READ_PAYLOAD,
  STATE_READ_ERROR,
  STATE_READ_SIZE
} UART_Read_State_E;

static std::unique_ptr<UART_Comm> _usb_uart;

bool init_uart()
{
  if (_usb_uart != NULL)
  {
    return true;
  }
  _usb_uart = std::make_unique<UART_Comm>();
  return _usb_uart->init();
}

UART_Comm& get_uart()
{
  assert(_usb_uart && "get_uart() before init_uart()");
  return *_usb_uart;
}

UART_Comm::UART_Comm()
: rx_buf(nullptr),
  thread_running(false),
  read_task(nullptr),
  write_task(nullptr)
{

}

UART_Comm::~UART_Comm()
{
  this->deinit();
}

bool UART_Comm::init()
{
  this->rx_buf = (uint8_t*)heap_caps_malloc(2048, MALLOC_CAP_DMA);
  if (this->rx_buf == nullptr)
  {
    ESP_LOGE(UART_Comm::TAG, "Could not allocate 2048 bytes of DMA memory");
    return false;
  }
  memset(this->rx_buf, 0, 2048);

  const uart_config_t uart_cfg = {
    .baud_rate             = 2000000,
    .data_bits             = UART_DATA_8_BITS,
    .parity                = UART_PARITY_DISABLE,
    .stop_bits             = UART_STOP_BITS_1,
    .flow_ctrl             = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh   = 0,
    .source_clk            = UART_SCLK_DEFAULT,
    .flags = {
      .allow_pd            = 1,
      .backup_before_sleep = 1
    }
  };
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 4096, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  return true;
}

void UART_Comm::deinit()
{

}

bool UART_Comm::start_thread()
{
  this->thread_running = true;
  if (this->read_task == nullptr)
  {
    BaseType_t rx_ret = xTaskCreatePinnedToCore(&UART_Comm::read_thread_entry, UART_Comm::TAG, 4096, this, 5, &this->read_task, tskNO_AFFINITY);
    if (rx_ret != pdPASS)
    {
      return false;
    }
  }

  if (this->write_task == nullptr)
  {
    BaseType_t tx_ret = xTaskCreatePinnedToCore(&UART_Comm::write_thread_entry, UART_Comm::TAG, 4096, this, 5, &this->write_task, tskNO_AFFINITY);
    if (tx_ret != pdPASS)
    {
      return false;
    }
  }
  
  return true;
}


// Private functions
void UART_Comm::read_thread_entry(void* pv)
{
  static_cast<UART_Comm*>(pv)->run_read_thread();
  vTaskDelete(nullptr);
}

void UART_Comm::write_thread_entry(void* pv)
{
  static_cast<UART_Comm*>(pv)->run_write_thread();
  vTaskDelete(nullptr);
}


void UART_Comm::run_read_thread()
{
  UART_Read_State_E state = STATE_READ_ADDR;
  uint8_t cmd = CMD_INVALID;
  for (;;)
  {
    int n = uart_read_bytes(UART_NUM_0, rx_buf, 2048, pdMS_TO_TICKS(50));
    if (n > 0)
    {
      ESP_LOGI(UART_Comm::TAG, "Read %d bytes", n);
      for (int i = 0; i < n; ++i)
      {
        switch (state)
        {
          case STATE_READ_ADDR:
          {
            if (rx_buf[i] == RPI5_ADDR)
            {
              state = STATE_READ_CMD;
            }
            break;
          }
          case STATE_READ_CMD:
          {
            switch (rx_buf[i])
            {
              case CMD_PING:
              {
                cmd = CMD_PING;
                state = STATE_READ_LEN;
                break;
              }
              case CMD_AUDIO_DATA:
              {
                cmd = CMD_AUDIO_DATA;
                state = STATE_READ_LEN;
                break;
              }
              default:
              {
                state = STATE_READ_ERROR;
                break;
              }
            }
            break;
          }
          case STATE_READ_LEN:
          {
            state = STATE_READ_PAYLOAD;
            break;
          }
          case STATE_READ_PAYLOAD:
          {
            state = STATE_READ_SIZE;
            break;
          }
          case STATE_READ_ERROR:
          case STATE_READ_SIZE:
          default:
          {
            state = STATE_READ_ADDR;
            break;
          }
        }
      }
    }
  }
}

void UART_Comm::run_write_thread()
{
  uint8_t seq = 0;
  for (;;)
  {
    CommPacketPing msg = {
      .header = {
        .type = REQUEST_PACKET,
        .addr = RPI5_ADDR,
        .cmd = CMD_PING,
        .len = sizeof(msg) - sizeof(CommPacketHeader)
      },
      .msg = 0x42,
      .seq = seq
    };
    uart_write_bytes(UART_NUM_0, &msg, sizeof(msg));
    seq += 2;
    ESP_LOGI(UART_Comm::TAG, "Wrote %d bytes", sizeof(msg));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}