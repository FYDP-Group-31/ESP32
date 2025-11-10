// C++
#include <cassert>

// Components
#include "adau1966a.hpp"
#include "uart_comm.hpp"

extern "C" {
  void app_main(void);
}

void app_main(void)
{
  assert(init_adau1966a(I2S_MCLK_GPIO, I2S_BCLK_GPIO, I2S_WS_GPIO, I2S_DOUT_GPIO));
  ADAU1966A& adau1966a = get_adau1966a();
  adau1966a.start_thread();

  assert(init_uart());
  UART_Comm& uart = get_uart();
  uart.start_thread();
}
