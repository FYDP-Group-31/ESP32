#pragma once

#define REQUEST_PACKET 0xAA
#define RESPONSE_PACKET 0x55

#define RPI5_ADDR 0x00
#define MCU_ADDR 0x01
#define INVALID_ADDR 0xFF

typedef enum : uint8_t {
    CMD_PING = 0x00U,
    CMD_AUDIO_DATA = 0x01U,
    CMD_RESET = 0x02U,
    CMD_POS = 0x03U,
    CMD_SIZE
} Command_E;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t addr; // Destination address
  Command_E cmd;
  uint16_t len;
} CommPacketHeader;

typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  uint16_t samples[256];
} CommPacketAudioData;

/**
 * Ping: RPi5 sends ping requests, MCU responds with ping responses
 */
typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  uint8_t msg;
  uint8_t seq; // Even number for requests, odd number for responses
} CommPacketPing;

typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  uint8_t pos;
  uint8_t depth;
  uint8_t seq;
} CommPacketPosRes;

typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  uint32_t wait_time_ms;
  uint8_t crc;
} CommPacketReset;