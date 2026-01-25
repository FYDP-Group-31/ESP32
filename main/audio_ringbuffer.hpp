#pragma once

#include "audio_defs.h"

class AudioRingBuffer {
  private:
    sample_t* buffer;
    size_t capacity;
    size_t head_idx;
    size_t tail_idx;
    size_t size;
  public:
    AudioRingBuffer(size_t capacity_samples);
    ~AudioRingBuffer();

    size_t read(sample_t* out_buf, size_t num_samples, size_t offset);
    size_t write(const sample_t* data, size_t num_samples);
    size_t get_free_size();
};
