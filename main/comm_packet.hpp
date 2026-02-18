#pragma once

#include "audio_defs.hpp"

typedef enum : uint8_t {
  REQUEST_PACKET = 0xAAU,
  RESPONSE_PACKET = 0x55U,
  INVALID_PACKET = 0xFFU
} PacketType_E;

typedef enum : uint8_t {
    RPI5_ADDR = 0x00U,
    MCU_ADDR = 0x01U,
    NUM_ADDR
} DeviceAddress_E;

typedef enum : uint8_t {
  CMD_PING = 0x00U,
  CMD_AUDIO_DATA = 0x01U,
  CMD_RESET = 0x02U,
  CMD_POS = 0x03U,
  CMD_SIZE
} Command_E;

typedef struct __attribute__((packed)) {
  PacketType_E type;
  DeviceAddress_E addr; // Destination address
  Command_E cmd;
  uint8_t len;
} CommPacketHeader;

typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  uint8_t msg;
  uint8_t seq; // Even number for requests, odd number for responses
} CommPacketPingReq;

typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  uint8_t curr_pos;
  uint8_t curr_depth;
  uint8_t seq; // Request seq + 1 (odd number)
} CommPacketPingRes;

typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  sample_t audio_data[128];
} CommPacketAudioData;

typedef struct __attribute__((packed)) { // RPi -> MCU
  CommPacketHeader header;
  uint8_t pos;
  uint8_t depth;
  uint8_t seq; // Even
} CommPacketPosReq;

typedef struct __attribute__((packed)) { // MCU -> RPi
  CommPacketHeader header;
  uint8_t pos;
  uint8_t depth;
  uint8_t seq; // Request seq + 1 (odd)
} CommPacketPosRes;

typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  uint32_t wait_time_ms;
} CommPacketResetReq;

typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  uint8_t reset_status;
} CommPacketResetRes;