#pragma once
#include <stdint.h>
#if defined(ARDUINO)
#include <Arduino.h>
#endif

// Source-audited Teensy 4.1 GPIOs only. Update this allowlist BEFORE wiring or
// enabling another peripheral. Do not sweep every board pin: SDIO (42..47),
// internal memory (48..54), USB/debug and active/reserved signals stay untouched.
// 20..23 remain reserved for RunCam/servos, even if temporarily compiled out.
constexpr uint8_t UNUSED_PINS[]={3,5,7,8,9,14,15,26,27,
                               30,31,32,33,34,35,36,37,38,39,40,41};
constexpr bool isUnusedPin(uint8_t pin){
  for(uint8_t unused:UNUSED_PINS)if(pin==unused)return true;
  return false;
}
#if defined(ARDUINO)
inline void sleepUnusedPins(){
  // Core-supported unused-pin mode: high impedance, no pull-up/down. This does
  // not power-gate the pin, attached hardware, CPU, peripheral clocks or USB.
  for(uint8_t pin:UNUSED_PINS)pinMode(pin,INPUT_DISABLE);
}
#endif
