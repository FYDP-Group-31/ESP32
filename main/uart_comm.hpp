#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include "gpio_defs.h"

#include "comm_packet.hpp"

class UART_Comm {
  private:
    uint8_t* rx_buf;

    bool thread_running;
    TaskHandle_t read_task;
    TaskHandle_t write_task;

    static void read_thread_entry(void* pv);
    static void write_thread_entry(void* pv);
    void run_read_thread();
    void run_write_thread();

  public:
    static constexpr const char* TAG = "UART";

    explicit UART_Comm();
    ~UART_Comm();

    bool init();
    void deinit();
    bool start_thread();
    void stop_thread();
};

bool init_uart();
UART_Comm& get_uart();
