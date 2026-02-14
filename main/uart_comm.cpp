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

static void restart_task(void* arg)
{
  uint32_t delay_ms = *reinterpret_cast<uint32_t*>(arg);
  vTaskDelay(pdMS_TO_TICKS(delay_ms));
  esp_restart();
}

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
  control_payload_buf(nullptr),
  audio_payload_buf(nullptr),
  curr_pos(0),
  curr_depth(0),
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

  this->control_payload_buf = (uint8_t*)heap_caps_malloc(256, MALLOC_CAP_DMA);
  if (this->control_payload_buf == nullptr)
  {
    ESP_LOGE("UART0", "Could not allocate 256 bytes of DMA memory");
    return false;
  }
  memset(this->control_payload_buf, 0, 256);

  this->audio_payload_buf = (uint8_t*)heap_caps_malloc(256, MALLOC_CAP_DMA);
  if (this->audio_payload_buf == nullptr)
  {
    ESP_LOGE("UART0", "Could not allocate 256 bytes of DMA memory");
    return false;
  }
  memset(this->audio_payload_buf, 0, 256);

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
  uint8_t payload_bytes_read = 0;
  CommPacketHeader header = {.type = INVALID_PACKET, .addr = NUM_ADDR, .cmd = CMD_SIZE, .len = 0};
  for (;;)
  {
    // ESP_LOGI("UART0", "sdfjsfhdsjkfds");
    int n = uart_read_bytes(UART_NUM_1, this->control_data_buf, 2048, pdMS_TO_TICKS(50));
    if (n > 0)
    {
      ESP_LOGI("UART1", "Read %d bytes", n);
      for (size_t i = 0; i < n; ++i)
      {
        uint8_t byte = this->control_data_buf[i];
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
            switch (byte)
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
              case CMD_RESET:
              {
                header.cmd = CMD_RESET;
                state = STATE_READ_LEN;
                break;
              }
              case CMD_AUDIO_DATA:
              default:
              {
                ESP_LOGE("UART1", "Invalid CMD type 0x%02X", byte);
                state = STATE_READ_ERROR;
                break;
              }
            }
            break;
          }
          case STATE_READ_LEN:
          {
            header.len = byte;
            if (header.len == 0)
            {
              state = STATE_READ_ERROR;
            }
            else
            {
              state = STATE_READ_PAYLOAD;
            }
            break;
          }
          case STATE_READ_PAYLOAD:
          {
            this->control_payload_buf[payload_bytes_read] = byte;
            ++payload_bytes_read;
            
            if (payload_bytes_read >= header.len)
            {
              ESP_LOGI("UART1",
                "Received packet:\n\
                \tType: 0x%02X\n\
                \tAddr: 0x%02X\n\
                \tCmd: 0x%02X\n\
                \tLen: %d bytes",
                header.type, header.addr, header.cmd, header.len
              );

              CommPacketHeader response_header = {
                .type = RESPONSE_PACKET,
                .addr = RPI5_ADDR,
                .cmd = header.cmd,
                .len = 0
              };

              switch (header.cmd)
              {
                case CMD_PING:
                {
                  CommPacketPingReq request = {
                    .header = header,
                    .msg = this->control_payload_buf[0],
                    .seq = this->control_payload_buf[1]
                  };
                  
                  response_header.len = sizeof(CommPacketPingRes) - sizeof(CommPacketHeader);
                  CommPacketPingRes response = {
                    .header = response_header,
                    .curr_pos = this->curr_pos,
                    .curr_depth = this->curr_depth,
                    .seq = static_cast<uint8_t>(request.seq + 1)
                  };
                  int bytes_written = uart_write_bytes(UART_NUM_1, (const char*)&response, sizeof(response));
                  if (bytes_written != sizeof(CommPacketHeader))
                  {
                    ESP_LOGE("UART1", "Failed to send ping response packet");
                  }
                  else
                  {
                    ESP_LOGI("UART1", "Sent ping response packet");
                  }
                  break;
                }
                case CMD_POS:
                {
                  CommPacketPosReq request = {
                    .header = header,
                    .pos = this->control_payload_buf[0],
                    .depth = this->control_payload_buf[1],
                    .seq = this->control_payload_buf[2]
                  };

                  this->curr_pos = request.pos;
                  this->curr_depth = request.depth;
                  response_header.len = sizeof(CommPacketPosRes) - sizeof(CommPacketHeader);
                  CommPacketPosRes response = {
                    .header = response_header,
                    .pos = this->curr_pos,
                    .depth = this->curr_depth,
                    .seq = static_cast<uint8_t>(request.seq + 1)
                  };
                  int bytes_written = uart_write_bytes(UART_NUM_1, (const char*)&response, sizeof(response));
                  if (bytes_written != sizeof(CommPacketHeader))
                  {
                    ESP_LOGE("UART1", "Failed to send position response packet");
                  }
                  else
                  {
                    ESP_LOGI("UART1", "Sent position response packet");
                  }
                  break;
                }
                case CMD_RESET:
                {
                  CommPacketResetReq request = {
                    .header = header,
                    .wait_time_ms = (static_cast<uint32_t>(this->control_payload_buf[0]) << 24) |
                                   (static_cast<uint32_t>(this->control_payload_buf[1]) << 16) |
                                   (static_cast<uint32_t>(this->control_payload_buf[2]) << 8) |
                                   (static_cast<uint32_t>(this->control_payload_buf[3]))
                  };

                  response_header.len = sizeof(CommPacketResetRes) - sizeof(CommPacketHeader);
                  CommPacketResetRes response = {
                    .header = response_header,
                    .reset_status = 0 // TODO: Define reset status codes
                  };
                  int bytes_written = uart_write_bytes(UART_NUM_1, (const char*)&response, sizeof(response));
                  if (bytes_written != sizeof(CommPacketHeader))
                  {
                    ESP_LOGE("UART1", "Failed to send reset response packet");
                  }
                  else      {
                    ESP_LOGI("UART1", "Sent reset response packet");
                  }
                  xTaskCreatePinnedToCore(&restart_task, "RestartTask", 64, (void*)request.wait_time_ms, 3, nullptr, tskNO_AFFINITY);
                  break;
                }
                case CMD_AUDIO_DATA:
                default:
                {
                  ESP_LOGW("UART1", "No response defined for command 0x%02X", header.cmd);
                  break;
                }
              }

              // Reset for next packet
              header.type = INVALID_PACKET;
              header.cmd = CMD_SIZE;
              header.addr = NUM_ADDR;
              header.len = 0;
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
            header.type = INVALID_PACKET;
            header.cmd = CMD_SIZE;
            header.addr = NUM_ADDR;
            header.len = 0;
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
  constexpr size_t PACKET_SIZE = sizeof(CommPacketAudioData);
  constexpr size_t PAYLOAD_SIZE = sizeof(CommPacketAudioData) - sizeof(CommPacketHeader);
  constexpr size_t NUM_SAMPLES = PAYLOAD_SIZE / sizeof(sample_t);
  
  size_t bytes_needed = PACKET_SIZE;
  size_t bytes_in_buffer = 0;
  uint8_t* packet_buf = this->audio_data_buf;  // Use first 260 bytes for current packet

  size_t frame_count = 0;
  
  for (;;)
  {
    // Read exactly the bytes we need to complete the packet
    int n = uart_read_bytes(UART_NUM_0, packet_buf + bytes_in_buffer, bytes_needed, pdMS_TO_TICKS(50));
    
    if (n > 0)
    {
      bytes_in_buffer += n;
      bytes_needed -= n;
      
      // Check if we have a complete packet
      if (bytes_needed == 0)
      {
        // Validate header
        if (
          packet_buf[0] == REQUEST_PACKET && 
          packet_buf[1] == MCU_ADDR && 
          packet_buf[2] == CMD_AUDIO_DATA && 
          packet_buf[3] == NUM_SAMPLES
        )
        {
          ++frame_count;
          // Convert samples from network order (big-endian) to host order
          sample_t* samples = (sample_t*)(packet_buf + 4);
          
          // Write audio data to DAC ring buffer
          if (!dac->write_to_ringbuf(samples, NUM_SAMPLES))
          {
            ESP_LOGW("UART0", "Failed to write audio data to DAC ring buffer");
          }
          else
          {
            ESP_LOGI("UART0", "Wrote %d samples to DAC ring buffer (%d)", NUM_SAMPLES, frame_count);
          }
        }
        else
        {
          ESP_LOGE(
            "UART0", "Invalid packet header: [0x%02X, 0x%02X, 0x%02X, 0x%02X]",
            packet_buf[0], packet_buf[1], packet_buf[2], packet_buf[3]
          );
        }
        
        // Reset for next packet
        bytes_in_buffer = 0;
        bytes_needed = PACKET_SIZE;
      }
    }
  }
}
