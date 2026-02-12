#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include "gpio_defs.h"

#include "comm_packet.hpp"

class UART_Comm {
  private:
    uint8_t* control_data_buf;
    uint8_t* audio_data_buf;
    uint8_t* control_payload_buf;
    uint8_t* audio_payload_buf;

    uint8_t curr_pos;
    uint8_t curr_depth;

    bool thread_running;
    TaskHandle_t control_data_recv_task;
    TaskHandle_t audio_data_recv_task;

    static void control_data_recv_thread_entry(void* pv);
    static void audio_data_recv_thread_entry(void* pv);
    void run_control_data_recv_thread(); // UART1
    void run_audio_data_recv_thread(); // UART0

  public:
    explicit UART_Comm();
    ~UART_Comm();

    bool init();
    void deinit();
    bool start_thread();
    void stop_thread();
};

bool init_uart();
UART_Comm& get_uart();
