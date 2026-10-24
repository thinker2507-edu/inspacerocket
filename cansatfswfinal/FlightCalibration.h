#pragma once
#include <stdint.h>
#include <math.h>

struct FlightCalibration {
  uint32_t generation=0;
  float pressure=0, gyroX=0, gyroY=0, gyroZ=0, verticalBias=0;
  constexpr bool valid() const {
    return isfinite(pressure) && pressure>2000 && pressure<120000 &&
      isfinite(gyroX) && fabsf(gyroX)<20 && isfinite(gyroY) && fabsf(gyroY)<20 &&
      isfinite(gyroZ) && fabsf(gyroZ)<20 && isfinite(verticalBias) && fabsf(verticalBias)<2;
  }
};
inline float relativePressureAltitude(float pressure,float reference) {
  if(!isfinite(pressure) || pressure<2000 || pressure>120000 ||
     !isfinite(reference) || reference<=2000 || reference>=120000)return NAN;
  return 44330.0f*(1.0f-powf(pressure/reference,0.190295f));
}

constexpr uint32_t recoverTelemetryUpperBound(uint32_t validHigh,uint32_t damagedSlots) {
  uint64_t upper=uint64_t(validHigh)+uint64_t(damagedSlots)*64;
  return upper>UINT32_MAX?UINT32_MAX:uint32_t(upper);
}

// Reserve BEFORE transmission. Reboot skips the unused portion of the last
// durable range. Sequence zero is never used; UINT32_MAX exhausts the namespace.
struct TelemetrySequence {
  static constexpr uint32_t BLOCK=64;
  uint32_t next=1,high=0;
  constexpr void restore(uint32_t durableHigh) {high=durableHigh;next=high==UINT32_MAX?0:high+1;}
  constexpr bool available() const {return next && next<=high;}
  constexpr uint32_t proposal() const {
    return !next?0:(uint64_t(next)+BLOCK-1>UINT32_MAX?UINT32_MAX:next+BLOCK-1);
  }
  constexpr void commit(uint32_t upper) {if(upper>=high)high=upper;}
  constexpr uint32_t take() {
    if(!available())return 0;
    uint32_t value=next;next=value==UINT32_MAX?0:value+1;return value;
  }
};
