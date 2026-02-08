#include "uart_comm.hpp"

#include <memory>
#include <cassert>

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "adau1966a.hpp"

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

static std::unique_ptr<UART_Comm> _uart;
static ADAU1966A* dac = nullptr;

bool init_uart()
{
  if (_uart != NULL)
  {
    return true;
  }
  _uart = std::make_unique<UART_Comm>();
  return _uart->init();
}

UART_Comm& get_uart()
{
  assert(_uart && "get_uart() before init_uart()");
  return *_uart;
}

UART_Comm::UART_Comm()
: control_data_buf(nullptr),
  audio_data_buf(nullptr),
  payload_buf(nullptr),
  thread_running(false),
  control_data_recv_task(nullptr),
  audio_data_recv_task(nullptr)
{

}

UART_Comm::~UART_Comm()
{
  this->deinit();
}

bool UART_Comm::init()
{
  dac = &get_adau1966a();
  if (dac == nullptr)
  {
    ESP_LOGE("UART Init", "Failed to get ADAU1966A instance");
    return false;
  }

  gpio_config_t uart_full_signal_gpio_cfg = {
    .pin_bit_mask = (1ULL << (UART0_FULL_GPIO - 1)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
    .hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&uart_full_signal_gpio_cfg));

  this->control_data_buf = (uint8_t*)heap_caps_malloc(2048, MALLOC_CAP_DMA);
  if (this->control_data_buf == nullptr)
  {
    ESP_LOGE("UART0", "Could not allocate 2048 bytes of DMA memory");
    return false;
  }
  memset(this->control_data_buf, 0, 2048);

  this->audio_data_buf = (uint8_t*)heap_caps_malloc(2048, MALLOC_CAP_DMA);
  if (this->audio_data_buf == nullptr)
  {
    ESP_LOGE("UART1", "Could not allocate 2048 bytes of DMA memory");
    return false;
  }
  memset(this->audio_data_buf, 0, 2048);

  this->payload_buf = (uint8_t*)heap_caps_malloc(2048, MALLOC_CAP_DMA);
  if (this->payload_buf == nullptr)
  {
    ESP_LOGE("UART0", "Could not allocate 2048 bytes of DMA memory");
    return false;
  }
  memset(this->payload_buf, 0, 2048);

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
    BaseType_t rx_ret = xTaskCreatePinnedToCore(&UART_Comm::control_data_recv_thread_entry, "UART0", 4096, this, 5, &this->control_data_recv_task, tskNO_AFFINITY);
    if (rx_ret != pdPASS)
    {
      return false;
    }
  }

  if (this->audio_data_recv_task == nullptr)
  {
    BaseType_t audio_rx_ret = xTaskCreatePinnedToCore(&UART_Comm::audio_data_recv_thread_entry, "UART1", 4096, this, 5, &this->audio_data_recv_task, tskNO_AFFINITY);
    if (audio_rx_ret != pdPASS)
    {
      return false;
    }
  }
  
  return true;
}

void UART_Comm::stop_thread()
{
  this->thread_running = false;
}


// Private functions
void UART_Comm::control_data_recv_thread_entry(void* pv)
{
  static_cast<UART_Comm*>(pv)->run_control_data_recv_thread();
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
  CommPacketHeader header = {.type = 0, .addr = 0, .cmd = CMD_SIZE, .len = 0};
  for (;;)
  {
    // ESP_LOGI("UART0", "sdfjsfhdsjkfds");
    int n = uart_read_bytes(UART_NUM_1, this->control_data_buf, 2048, pdMS_TO_TICKS(50));
    if (n > 0)
    {
      ESP_LOGI("UART1", "Read %d bytes", n);
      for (size_t i = 0; i < n; ++i)
      {
        switch (state)
        {
          case STATE_READ_START:
          {
            if (this->control_data_buf[i] == REQUEST_PACKET)
            {
              header.type = REQUEST_PACKET;
              state = STATE_READ_ADDR;
            }
            break;
          }
          case STATE_READ_ADDR:
          {
            if (this->control_data_buf[i] == MCU_ADDR)
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
            switch (this->control_data_buf[i])
            {
              case CMD_PING:
              {
                header.cmd = CMD_PING;
                state = STATE_READ_LEN;
                break;
              }
              case CMD_POS:
              {
                header.cmd = CMD_POS;
                state = STATE_READ_LEN;
                break;
              }
              case CMD_AUDIO_DATA:
              case CMD_RESET: // TODO: Implement
              default:
              {
                ESP_LOGE("UART1", "Invalid CMD type 0x%02X", this->control_data_buf[i]);
                state = STATE_READ_ERROR;
                break;
              }
            }
            break;
          }
          case STATE_READ_LEN:
          {
            len |= (this->control_data_buf[i] << (8 * len_bytes_read));
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
            this->payload_buf[payload_bytes_read] = this->control_data_buf[i];
            ++payload_bytes_read;
            
            if (payload_bytes_read >= len)
            {
              ESP_LOGI("UART1",
                "Received packet:\n\
                \tType: 0x%02X\n\
                \tAddr: 0x%02X\n\
                \tCmd: 0x%02X\n\
                \tLen: %d bytes",
                header.type, header.addr, header.cmd, header.len
              );

              // if (header.cmd == CMD_AUDIO_DATA)
              // {
              //   // Write audio data to DAC ring buffer
              //   size_t num_samples = header.len / sizeof(sample_t);
              //   sample_t* audio_samples = (sample_t*)this->payload_buf;
              //   if (!dac->write_to_ringbuf(audio_samples, num_samples))
              //   {
              //     ESP_LOGE("UART1", "Failed to write audio data to DAC ring buffer");
              //   }
              // }

              send_response(header);

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
            ESP_LOGE("UART1", "Error in receiving packet, resetting state machine");
            // Reset for next packet
            len = 0;
            len_bytes_read = 0;
            payload_bytes_read = 0;
            state = STATE_READ_START;

            --i;
            break;
          }
        }
      }
    }
  }
}

void UART_Comm::run_audio_data_recv_thread()
{
  UART_Read_State_E state = STATE_READ_START;
  uint8_t len_bytes_read = 0; // Used to track how many length bytes have been read
  uint16_t payload_bytes_read = 0;
  CommPacketHeader header = {.type = 0, .addr = 0, .cmd = CMD_SIZE, .len = 0};
  for (;;)
  {
    int n = uart_read_bytes(UART_NUM_0, this->audio_data_buf, 2048, pdMS_TO_TICKS(50));
    if (n > 0)
    {
      ESP_LOGI("UART0", "Audio Data Thread: Read %d bytes", n);

      for (size_t i = 0; i < n; ++i)
      {
        uint8_t byte = this->audio_data_buf[i];
        switch (state)
        {
          case STATE_READ_START:
          {
            if (byte == REQUEST_PACKET)
            {
              header.type = REQUEST_PACKET;
              state = STATE_READ_ADDR;
            }
            break;
          }
          case STATE_READ_ADDR:
          {
            if (byte == MCU_ADDR)
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
            if (byte == CMD_AUDIO_DATA)
            {
              header.cmd = CMD_AUDIO_DATA;
              state = STATE_READ_LEN;
            }
            else
            {
              ESP_LOGE("UART0", "Invalid CMD type 0x%02X", byte);
              state = STATE_READ_ERROR;
            }
            break;
          }
          case STATE_READ_LEN:
          {
            header.len |= (byte << (8 * len_bytes_read));
            ++len_bytes_read;
            if (len_bytes_read == 2)
            {
              len_bytes_read = 0;
              if (header.len == 0)
              {
                state = STATE_READ_ERROR;
              }
              else
              {
                state = STATE_READ_PAYLOAD;
              }
            }
            break;
          }
          case STATE_READ_PAYLOAD:
          {
            if (payload_bytes_read == header.len)
            {
              payload_bytes_read = 0;
              state = STATE_READ_START;
            }
            else
            {
              ++payload_bytes_read;
            }
            break;
          }
          case STATE_READ_ERROR:
          case STATE_READ_SIZE:
          default:
          {
            ESP_LOGE("UART0", "Error in receiving packet, resetting state machine");
            // Reset state machine
            memset(&header, 0, sizeof(CommPacketHeader));
            state = STATE_READ_START;
            --i;
            break;
          }
        }
      }
    }
  }
}

void UART_Comm::send_response(const CommPacketHeader& req_header)
{
  CommPacketHeader resp_header;
  resp_header.type = RESPONSE_PACKET;
  resp_header.addr = RPI5_ADDR;
  resp_header.cmd = req_header.cmd;
  resp_header.len = 0;

  bool send_data = false;

  switch (req_header.cmd)
  {
    case CMD_PING:
    {
      // Prepare ping response packet
      resp_header.len = sizeof(CommPacketPing) - sizeof(CommPacketHeader);
      send_data = true;
      break;
    }
    case CMD_POS:
    {
      resp_header.len = sizeof(CommPacketPosRes) - sizeof(CommPacketHeader);
      send_data = true;
    }
    case CMD_AUDIO_DATA:
    {
      // No response needed for audio data
      break;
    }
    default:
    {
      ESP_LOGW("UART1", "No response defined for command 0x%02X", req_header.cmd);
      return;
    }
  } 
  if (send_data == true)
  {
    int bytes_written = uart_write_bytes(UART_NUM_1, (const char*)&resp_header, sizeof(CommPacketHeader));
    if (bytes_written != sizeof(CommPacketHeader))
    {
        ESP_LOGE("UART1", "Failed to send response packet");
    }
    else
    {
        ESP_LOGI("UART1", "Sent response packet");
    }
  }
  return;
}

void UART_Comm::signal_uart_full()
{
  gpio_set_level(UART0_FULL_GPIO, 0);
}

void UART_Comm::signal_uart_empty()
{
  gpio_set_level(UART0_FULL_GPIO, 1);
}
