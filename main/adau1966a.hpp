#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_tdm.h"
#include "driver/i2s_types.h"
#include "driver/i2s_common.h"
#include "driver/i2c_master.h"

#include "esp_async_memcpy.h"

#include "gpio_defs.h"
#include "audio_defs.hpp"

class ADAU1966A {
  private:
    gpio_num_t mclk_gpio;
    gpio_num_t bclk_gpio;
    gpio_num_t ws_gpio;
    gpio_num_t data_gpio;

    // ********** DRIVER HANDLES **********
    i2s_chan_handle_t i2s_driver;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    // ********** INTERNAL BUFFERS **********
    // NOTE: Delay calculations should be done between chunks
    // NOTE: Multiple chunks may be needed to satisfy delay requirements

    // Buffer to store frame data required for delay filter
    // Ringbuffer items are copied to this buffer before applying delay filter
    sample_t* sliding_window_buf;
    size_t sliding_window_read_idx; // Global read index for channels with 0 phase offset
    size_t sliding_window_write_idx; // Index of the next sample to write in sliding window buffer
    size_t sliding_window_channel_idx[TDM_SLOTS];
    size_t sliding_window_remaining_size;
    int32_t channel_delay_offset[TDM_SLOTS];

    // Buffer to store individual channel frames in contiguous memory
    // Frames are moved to channel_frame_buf after applying delay filter
    // Size should be FRAMES_PER_CHUNK * sizeof(sample_t)
    sample_t** channel_frame_buf;

    // Buffer to store single shot of TDM16 i2s_channel_write
    // Size should be FRAMES_PER_CHUNK * TDM_SLOTS * sizeof(sample_t)
    // Frames are copied from channel_frame_buf to this buffer before writing to I2S
    sample_t* chunk;
    // ***************************************

    bool thread_running;
    TaskHandle_t task;

    static void thread_entry(void* pv);
    void run_thread();
    void signal_ringbuf_full();
    void signal_ringbuf_ready();

    void setup_dac();

  public:
    static constexpr const char* TAG = "ADAU1966A";

    explicit ADAU1966A(
      gpio_num_t mclk_gpio,
      gpio_num_t bclk_gpio,
      gpio_num_t ws_gpio,
      gpio_num_t data_gpio
    );
    ~ADAU1966A();

    bool init();
    void deinit();
    bool start_thread();
    void stop_thread();

    size_t read_ringbuf(size_t num_samples, int32_t offset, sample_t* out_buf);
    bool write_to_ringbuf(const sample_t* data, size_t num_samples);
    size_t get_ringbuf_free_size();

    void set_channel_integer_delay_offset(uint8_t channel, int32_t offset);
};

bool init_adau1966a(gpio_num_t mclk_gpio, gpio_num_t bclk_gpio, gpio_num_t ws_gpio, gpio_num_t data_gpio);
ADAU1966A& get_adau1966a();
