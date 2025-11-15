#pragma once

#define RPI5_ADDR 0x00
#define MCU_ADDR 0x01
#define INVALID_ADDR 0xFF

#define CMD_PING 0x00
#define CMD_AUDIO_DATA 0x01
#define CMD_INVALID 0xFF

typedef struct __attribute__((packed)) {
  uint8_t addr;
  uint8_t cmd;
  uint16_t len;
} CommPacketHeader;

typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  uint16_t samples[256];
} CommPacketAudioData;

typedef struct __attribute__((packed)) {
  CommPacketHeader header;
  uint8_t msg;
} CommPacketPing;