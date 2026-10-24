#pragma once
#include <stdint.h>
#if defined(ARDUINO)
#include <Arduino.h>
#endif

// Independent CanSat allowlist based on current source. Do not assume an
// undocumented PSMS interface: reserve 20..23 and update this file when its
// actual wiring is established. Never touch SDIO 42..47, internal memory 48..54,
// USB/debug or the existing radio, flash, sensor and GNSS signals.
constexpr uint8_t UNUSED_PINS[]={3,5,7,8,9,14,15,26,27,
                               30,31,32,33,34,35,36,37,38,39,40,41};
constexpr bool isUnusedPin(uint8_t pin){
  for(uint8_t unused:UNUSED_PINS)if(pin==unused)return true;
  return false;
}
#if defined(ARDUINO)
inline void sleepUnusedPins(){
  // High impedance, no pulls. Not MCU sleep or peripheral power gating.
  for(uint8_t pin:UNUSED_PINS)pinMode(pin,INPUT_DISABLE);
}
#endif
