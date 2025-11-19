#include "uart_comm.hpp"

#include <memory>
#include <cassert>

#include "esp_log.h"
#include "esp_heap_caps.h"

typedef enum {
  // STATE_READ_IDLE,
  STATE_READ_START,
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
  audio_data_buf(nullptr),
  thread_running(false),
  control_data_recv_task(nullptr),
  write_task(nullptr),
  audio_data_recv_task(nullptr)
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

  this->audio_data_buf = (uint8_t*)heap_caps_malloc(2048, MALLOC_CAP_DMA);
  if (this->audio_data_buf == nullptr)
  {
    ESP_LOGE(UART_Comm::TAG, "Could not allocate 2048 bytes of DMA memory for audio data");
    return false;
  }
  memset(this->audio_data_buf, 0, 2048);

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
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART0_TX_GPIO, UART0_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 4096, 4096, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART1_TX_GPIO, UART1_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  return true;
}

void UART_Comm::deinit()
{

}

bool UART_Comm::start_thread()
{
  this->thread_running = true;
  if (this->control_data_recv_task == nullptr)
  {
    BaseType_t rx_ret = xTaskCreatePinnedToCore(&UART_Comm::control_data_recv_thread_entry, UART_Comm::TAG, 4096, this, 5, &this->control_data_recv_task, tskNO_AFFINITY);
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

  if (this->audio_data_recv_task == nullptr)
  {
    BaseType_t audio_rx_ret = xTaskCreatePinnedToCore(&UART_Comm::audio_data_recv_thread_entry, UART_Comm::TAG, 4096, this, 5, &this->audio_data_recv_task, tskNO_AFFINITY);
    if (audio_rx_ret != pdPASS)
    {
      return false;
    }
  }
  
  return true;
}


// Private functions
void UART_Comm::control_data_recv_thread_entry(void* pv)
{
  static_cast<UART_Comm*>(pv)->run_control_data_recv_thread();
  vTaskDelete(nullptr);
}

void UART_Comm::write_thread_entry(void* pv)
{
  static_cast<UART_Comm*>(pv)->run_write_thread();
  vTaskDelete(nullptr);
}

void UART_Comm::audio_data_recv_thread_entry(void* pv)
{
  static_cast<UART_Comm*>(pv)->run_audio_data_recv_thread();
  vTaskDelete(nullptr);
}

void UART_Comm::run_control_data_recv_thread()
{
  UART_Read_State_E state = STATE_READ_START;
  uint16_t len = 0;
  uint8_t len_bytes_read = 0;
  uint16_t payload_bytes_read = 0;
  CommPacketHeader header = {.type = 0, .addr = 0, .cmd = CMD_INVALID, .len = 0};
  for (;;)
  {
    int n = uart_read_bytes(UART_NUM_0, this->rx_buf, 2048, pdMS_TO_TICKS(50));
    if (n > 0)
    {
      ESP_LOGI(UART_Comm::TAG, "Read %d bytes", n);
      for (int i = 0; i < n; ++i)
      {
        switch (state)
        {
          case STATE_READ_START:
          {
            if (rx_buf[i] == REQUEST_PACKET)
            {
              // ESP_LOGI(UART_Comm::TAG, "Request packet received");
              header.type = REQUEST_PACKET;
              state = STATE_READ_ADDR;
            }
            else if (rx_buf[i] == RESPONSE_PACKET)
            {
              // ESP_LOGI(UART_Comm::TAG, "Response packet received");
              header.type = RESPONSE_PACKET;
              state = STATE_READ_ADDR;
            }
            else
            {
              ESP_LOGW(UART_Comm::TAG, "Invalid packet type received: 0x%02X", rx_buf[i]);
              state = STATE_READ_ERROR;
            }
            break;
          }
          case STATE_READ_ADDR:
          {
            if (rx_buf[i] == MCU_ADDR)
            {
              header.addr = MCU_ADDR;
              state = STATE_READ_CMD;
            }
            else
            {
              state = STATE_READ_START;
            }
            break;
          }
          case STATE_READ_CMD:
          {
            switch (rx_buf[i])
            {
              case CMD_PING:
              {
                // ESP_LOGI(UART_Comm::TAG, "CMD_PING received");
                header.cmd = CMD_PING;
                state = STATE_READ_LEN;
                break;
              }
              case CMD_AUDIO_DATA:
              {
                // ESP_LOGI(UART_Comm::TAG, "CMD_AUDIO_DATA received");
                header.cmd = CMD_AUDIO_DATA;
                state = STATE_READ_LEN;
                break;
              }
              default:
              {
                // ESP_LOGI(UART_Comm::TAG, "Invalid command received: 0x%02X", rx_buf[i]);
                state = STATE_READ_ERROR;
                break;
              }
            }
            break;
          }
          case STATE_READ_LEN:
          {
            len |= (rx_buf[i] << (8 * len_bytes_read));
            ++len_bytes_read;
            if (len_bytes_read >= 2)
            {
              len_bytes_read = 0;
              if (len == 0)
              {
                state = STATE_READ_ERROR;
              }
              else
              {
                header.len = len;
                state = STATE_READ_PAYLOAD;
              }
            }
            break;
          }
          case STATE_READ_PAYLOAD:
          {
            ++payload_bytes_read;
            
            if (payload_bytes_read >= len)
            {
              ESP_LOGI(UART_Comm::TAG,
                "Received packet:\n\
                \tType: 0x%02X\n\
                \tAddr: 0x%02X\n\
                \tCmd: 0x%02X\n\
                \tLen: %d bytes",
                header.type, header.addr, header.cmd, header.len
              );
              // Reset for next packet
              len = 0;
              len_bytes_read = 0;
              payload_bytes_read = 0;
              state = STATE_READ_START;
            }
            break;
          }
          case STATE_READ_ERROR:
          case STATE_READ_SIZE:
          default:
          {
            state = STATE_READ_START;
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
    uart_write_bytes(UART_NUM_1, &msg, sizeof(msg));
    seq += 2;
    ESP_LOGI(UART_Comm::TAG, "Wrote %d bytes", sizeof(msg));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void UART_Comm::run_audio_data_recv_thread()
{
  for (;;)
  {
    int n = uart_read_bytes(UART_NUM_1, this->audio_data_buf, 2048, pdMS_TO_TICKS(50));
    if (n > 0)
    {
      ESP_LOGI(UART_Comm::TAG, "Audio Data Thread: Read %d bytes", n);
    }
  }
}