#include "uart_comm.hpp"

#include <memory>
#include <cassert>

#include "esp_log.h"
#include "esp_heap_caps.h"

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
    .baud_rate             = 115200,
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

void UART_Comm::stop_thread()
{
  this->thread_running = false;
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
  for (;;)
  {
    int n = uart_read_bytes(UART_NUM_0, rx_buf, 2048, pdMS_TO_TICKS(50));
    if (n > 0)
    {
      ESP_LOGI(UART_Comm::TAG, "Read %d bytes", n);
    }
  }
}

void UART_Comm::run_write_thread()
{
  // int i = 0;
  for (;;)
  {
    // char msg[64];
    // int n = snprintf(msg, sizeof(msg), "Hello from ESP32-P4 %d\r\n", i++);
    CommPacket msg = {1,2,3};
    uart_write_bytes(UART_NUM_0, &msg, sizeof(msg));        // queues to TX buffer/FIFO
    ESP_LOGI(UART_Comm::TAG, "Wrote %d bytes", sizeof(msg));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}