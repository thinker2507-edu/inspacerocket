#pragma once
#include <Arduino.h>
#include <SPI.h>

// User marking "25Q64jvs10": W25Q64JV, 64 Mbit (8 MiB), 2.7..3.6 V.
// Uses the project's existing CS10/default SPI. Confirm physical wiring and 3.3 V.
// Dedicated recovery journal (not the routine flight log, which is on SD).
// Existing records are retained; foreign contents inhibit writes. NEVER erase.
#ifndef FLIGHT_FLASH_ENABLED
#define FLIGHT_FLASH_ENABLED 1
#endif
#ifndef FLIGHT_FLASH_JEDEC
#define FLIGHT_FLASH_JEDEC 0xEF4017UL
#endif
#ifndef FLIGHT_FLASH_START
#define FLIGHT_FLASH_START 0UL
#endif
#ifndef FLIGHT_FLASH_BYTES
#define FLIGHT_FLASH_BYTES (8UL*1024UL*1024UL)
#endif
#if FLIGHT_FLASH_ENABLED
#include <SerialFlash.h>
static_assert(FLIGHT_FLASH_JEDEC && FLIGHT_FLASH_BYTES >= 8192 &&
              FLIGHT_FLASH_START % 256 == 0 && FLIGHT_FLASH_BYTES % 256 == 0,
              "Confirm JEDEC ID and dedicated log region before enabling flash");
#endif

struct FlashHardware {
  bool enabled = false;
  uint32_t detectedId = 0;
  uint8_t chipSelect=10;
  uint32_t size() const { return FLIGHT_FLASH_BYTES; }
  bool begin(uint8_t cs) {
#if FLIGHT_FLASH_ENABLED
    chipSelect=cs;
    pinMode(cs,OUTPUT);digitalWrite(cs,HIGH);
    uint8_t id[5] = {};
    // Probe before library initialization (other chip types can change address
    // modes in begin()). No configuration write is made to an unidentified part.
    SPI.beginTransaction(SPISettings(10000000,MSBFIRST,SPI_MODE0));
    digitalWrite(cs,LOW);SPI.transfer(0x9F);
    for(uint8_t i=0;i<3;++i)id[i]=SPI.transfer(0);
    digitalWrite(cs,HIGH);SPI.endTransaction();
    uint32_t jedec = (uint32_t(id[0])<<16) | (uint32_t(id[1])<<8) | id[2];
    detectedId=jedec;
    if (jedec != FLIGHT_FLASH_JEDEC ||
        uint64_t(FLIGHT_FLASH_START)+FLIGHT_FLASH_BYTES > SerialFlash.capacity(id)) return false;
    if (!SerialFlash.begin(SPI, cs)) return false;
    enabled = true;
#else
    (void)cs;
#endif
    return enabled;
  }
  bool ready() {
#if FLIGHT_FLASH_ENABLED
    if(!enabled)return false;
    // SerialFlash's RAM busy flag resets with the MCU. Always poll real WIP,
    // including on recovery, before allowing reads or writes to a powered chip.
    SPI.beginTransaction(SPISettings(10000000,MSBFIRST,SPI_MODE0));
    digitalWrite(chipSelect,LOW);SPI.transfer(0x05);uint8_t status=SPI.transfer(0);
    digitalWrite(chipSelect,HIGH);SPI.endTransaction();
    return !(status&1) && SerialFlash.ready();
#else
    return false;
#endif
  }
  bool read(uint32_t address, void* data, uint32_t n) {
#if FLIGHT_FLASH_ENABLED
    if (!ready() || uint64_t(address)+n > size()) return false;
    SerialFlash.read(FLIGHT_FLASH_START+address, data, n);
    return true;
#else
    (void)address; (void)data; (void)n; return false;
#endif
  }
  bool program(uint32_t address, const void* data, uint32_t n) {
#if FLIGHT_FLASH_ENABLED
    // ready() first + one page only avoids SerialFlash's internal wait/page loop.
    if (!ready() || !n || n > 128 || uint64_t(address)+n > size() ||
        ((FLIGHT_FLASH_START+address)&255)+n > 256) return false;
    SerialFlash.write(FLIGHT_FLASH_START+address, data, n);
    return true;
#else
    (void)address; (void)data; (void)n; return false;
#endif
  }
};
