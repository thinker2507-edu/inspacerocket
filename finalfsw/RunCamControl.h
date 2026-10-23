#pragma once
#include <stdint.h>
#include <stddef.h>

// Recording-toggle experiment: both intents use the reference POWER BUTTON operation.
// This is not a verified recording command. Keep these aliases easy to change.
#define CAM_ACT_POWER        0x01
#define CAM_ACT_REC_START    0x01
#define CAM_ACT_REC_STOP     0x01

constexpr uint8_t crc8DvbS2(const uint8_t* d, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

constexpr bool buildRunCamPacket(uint8_t action, uint8_t (&p)[4]) {
  if (action != CAM_ACT_POWER && action != CAM_ACT_REC_START && action != CAM_ACT_REC_STOP) return false;
  p[0] = 0xCC; p[1] = 0x01; p[2] = action;
  p[3] = crc8DvbS2(p, 3);
  return true;
}
