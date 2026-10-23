#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "FlightCalibration.h"
#if __cplusplus >= 202002L
#include <array>
#include <bit>
#define FLIGHT_CX constexpr
#else
#define FLIGHT_CX
#endif

enum RecordType : uint8_t { LOG_RESET=1, LOG_ARM, LOG_LAUNCH, LOG_STATE, LOG_SAMPLE,
  LOG_CAM_INTENT, LOG_CAM_RESULT, LOG_COMPLETE, LOG_RECOVERY, LOG_CLOCK,
  LOG_CALIBRATION, LOG_PACKET_RANGE, LOG_COUNTER_RESET, LOG_MISSION_RESET };
enum RecordFlags : uint32_t {
  RF_LAUNCHED=1, RF_COMPLETE=2, RF_ARMED=4, RF_RTC_VALID=8, RF_RTC_TRUSTED=16,
  RF_INA_VALID=32, RF_CAM_ACTIVE=64, RF_START_SENT=128, RF_STOP_INTENT=256,
  RF_STOP_SENT=512, RF_RECOVERED=1024, RF_INHIBIT_CAMERA=2048, RF_STORAGE_FAULT=4096,
  RF_CMD_SEQ=8192, RF_RECOVERY_HOLD=16384, RF_START_REQUESTED=32768,
  RF_BARO_VALID=65536, RF_IMU_VALID=131072, RF_COUNTER_UNKNOWN=262144,
  RF_SERVO_INTENT=524288, RF_SERVO_COMMANDED=1048576
};
enum TimeQuality : uint8_t { TIME_UNSTARTED, TIME_MONOTONIC, TIME_RTC_RECOVERED, TIME_UNCERTAIN };
constexpr uint64_t UNKNOWN_ELAPSED = UINT64_MAX;
constexpr uint32_t RECORD_MAGIC = 0x314C4652; // RFL1, little-endian
struct __attribute__((packed)) FlightRecord {
  uint32_t magic, sequence, mission, bootMs, rtcUnix, launchRtc;
  uint64_t elapsed;
  uint8_t version, type, state, quality;
  uint32_t flags;
  float groundPressure, altitude, velocity, maxAltitude, maxVelocity;
  float ax, ay, az, voltage, shunt_mV, current_mA, power_mW;
  uint32_t commandSequence, landingRtc;
  uint64_t landingElapsed;
  uint32_t resetCause, dropped;
  struct __attribute__((packed)) {
    uint8_t detail; // same offset as version-1 reserved[0] camera intent
    uint16_t length;
    uint32_t telemetryHigh, calibrationGeneration;
  } metadata;
  uint32_t crc;
  uint8_t commit;
};
static_assert(sizeof(FlightRecord)==128 && offsetof(FlightRecord,crc)==123, "Journal layout changed: bump version");
constexpr uint32_t flightCrcBytes(const uint8_t* p, size_t n) {
  uint32_t crc=0xFFFFFFFFUL;
  while(n--) { crc^=*p++; for(int b=0;b<8;++b) crc=(crc>>1)^(0xEDB88320UL & (0UL-(crc&1))); }
  return ~crc;
}
inline uint32_t flightCrc(const void* data,size_t n){return flightCrcBytes((const uint8_t*)data,n);}
FLIGHT_CX inline uint32_t recordCrc(const FlightRecord& r) {
#if __cplusplus >= 202002L
  auto bytes=std::bit_cast<std::array<uint8_t,128>>(r);
  return flightCrcBytes(bytes.data(),offsetof(FlightRecord,crc));
#else
  return flightCrc(&r,offsetof(FlightRecord,crc));
#endif
}
constexpr bool calibrationPayloadValid(const FlightRecord& r) {
  if(r.type!=LOG_CALIBRATION)return true;
  FlightCalibration c{r.metadata.calibrationGeneration,r.groundPressure,r.ax,r.ay,r.az,r.voltage};
  return r.version==2 && c.generation && c.valid();
}
FLIGHT_CX inline bool validRecord(const FlightRecord& r) {
  return r.commit==0xA5 && r.magic==RECORD_MAGIC &&
         (r.version==1 || (r.version==2 && r.metadata.length==sizeof(r))) && r.state<=6 &&
         r.type>=LOG_RESET && r.type<=LOG_MISSION_RESET &&
         r.quality<=TIME_UNCERTAIN && r.crc==recordCrc(r) && calibrationPayloadValid(r);
}
FLIGHT_CX inline bool erasedRecord(const FlightRecord& r) {
#if __cplusplus >= 202002L
  auto bytes=std::bit_cast<std::array<uint8_t,128>>(r);
  const uint8_t* p=bytes.data();
#else
  const uint8_t* p=(const uint8_t*)&r;
#endif
  for(size_t i=0;i<sizeof(r);++i) if(p[i]!=0xFF) return false;
  return true;
}
FLIGHT_CX inline bool sameRecordBody(const FlightRecord& a,const FlightRecord& b) {
#if __cplusplus >= 202002L
  auto aa=std::bit_cast<std::array<uint8_t,128>>(a),bb=std::bit_cast<std::array<uint8_t,128>>(b);
  for(size_t i=0;i<127;++i)if(aa[i]!=bb[i])return false;
  return true;
#else
  return !memcmp(&a,&b,127);
#endif
}

// A fresh unarmed mission checkpoint; does not reset calibration or packet IDs.
constexpr FlightRecord padMissionResetRecord(FlightRecord r,uint32_t mission){
  r.type=LOG_MISSION_RESET;r.mission=mission;r.state=0;r.quality=TIME_UNSTARTED;r.elapsed=0;
  r.launchRtc=r.landingRtc=0;r.landingElapsed=UNKNOWN_ELAPSED;
  r.flags&=RF_RTC_VALID|RF_RTC_TRUSTED|RF_INA_VALID|RF_CMD_SEQ|RF_IMU_VALID|RF_COUNTER_UNKNOWN;
  r.altitude=r.velocity=NAN;r.maxAltitude=r.maxVelocity=0;r.metadata.detail=0;
  return r;
}
// Backend is deliberately tiny: size, begin, ready, read and one-page program.
// One data read/program per service call, with bounded status polls. Recovery
// reads 4 KiB at once (32 records); normal writes remain one-page operations.
// Never stop at an erased slot: later valid records must not be missed. Never erase.
// Body -> verify ->
// separate commit byte -> verify. Torn slots are consumed, never reused.
template<class Backend> class FlightJournal {
public:
  enum Status : uint8_t { UNAVAILABLE, SCANNING, READY, FAULT, FULL, FOREIGN };
  Backend& io;
  Status status=UNAVAILABLE;
  FlightRecord recovered={};
  FlightRecord calibration={};
  bool haveCalibration=false, haveLegacy=false;
  uint32_t telemetryHigh=0, damagedSlots=0, counterEpoch=0;
  bool haveRecord=false, tailTorn=false;
  bool criticalLost=false;
  uint32_t dropped=0, nextSequence=1, committed=0;
  static constexpr uint32_t RECOVERY_RECORDS=32;
  FLIGHT_CX uint32_t scannedBytes() const { return scanAddress; }
  FLIGHT_CX explicit FlightJournal(Backend& backend):io(backend) {}
  FLIGHT_CX void begin(uint8_t cs, uint32_t now) {
    if(io.begin(cs)) { status=SCANNING; busySince=now; }
  }
  FLIGHT_CX bool scanning() const { return status==SCANNING; }
  FLIGHT_CX bool usable() const { return status==READY; }
  FLIGHT_CX bool idle() const { return usable() && !count && !phase; }
  FLIGHT_CX uint32_t append(FlightRecord r, bool critical=true) {
    if(!usable()) { ++dropped; if(critical)criticalLost=true; return 0; }
    uint32_t slots=(io.size()-nextAddress)/128;
    // Preserve final 32 slots for critical events, not routine samples.
    if(!critical && (slots<=32UL+count || count>=6)) { ++dropped; return 0; }
    if(slots<=count || count==8) {
      int victim=-1;
      // Never evict the head while its body/commit is being programmed.
      if(critical) for(uint8_t i=phase?1:0;i<count;++i) if(queue[i].type==LOG_SAMPLE){victim=i;break;}
      if(victim<0){++dropped;if(critical)criticalLost=true;return 0;}
      for(uint8_t i=victim;i+1<count;++i)queue[i]=queue[i+1];
      --count; ++dropped;
    }
    r.magic=RECORD_MAGIC; r.version=2; r.metadata.length=sizeof(r);r.sequence=nextSequence++;
    r.dropped=dropped; r.crc=recordCrc(r); r.commit=0xFF;
    queue[count++]=r;
    return r.sequence;
  }
  FLIGHT_CX void service(uint32_t now) {
    if(status!=SCANNING && status!=READY)return;
    if(!io.ready()) { if(uint32_t(now-busySince)>100)status=FAULT; return; }
    busySince=now;
    if(status==SCANNING) {
      if(scanAddress>=io.size()) {
        status=nextAddress>=io.size()?FULL:READY; return;
      }
      uint32_t bytes=io.size()-scanAddress;
      if(bytes>sizeof(scanBuffer))bytes=sizeof(scanBuffer);
      if(bytes%sizeof(FlightRecord) || !io.read(scanAddress,scanBuffer,bytes)){status=FAULT;return;}
      for(uint32_t i=0;i<bytes/sizeof(FlightRecord);++i) {
        const FlightRecord& r=scanBuffer[i];
        if(!erasedRecord(r)) {
          nextAddress=scanAddress+128;
          if(validRecord(r)) {
            if(r.type==LOG_COUNTER_RESET && r.sequence>counterEpoch){
              counterEpoch=r.sequence;telemetryHigh=0;damagedSlots=0;haveLegacy=false;
            }
            if(r.version==1 || (r.flags&RF_COUNTER_UNKNOWN))haveLegacy=true;
            if(r.version==2) {
              if(r.metadata.telemetryHigh>telemetryHigh)telemetryHigh=r.metadata.telemetryHigh;
              if(r.type==LOG_CALIBRATION && (!haveCalibration || r.sequence>calibration.sequence)) {
                calibration=r;haveCalibration=true;
              }
            }
            if(!haveRecord || r.sequence>recovered.sequence){
              recovered=r;haveRecord=true;tailTorn=false;nextSequence=r.sequence+1;
            }
          } else {
            tailTorn=true;++damagedSlots;
            if(r.magic!=RECORD_MAGIC && r.magic!=0xFFFFFFFFUL){status=FOREIGN;return;}
          }
        }
        scanAddress+=sizeof(FlightRecord);
      }
      if(scanAddress>=io.size())status=nextAddress>=io.size()?FULL:READY;
      return;
    }
    if(!count)return;
    if(phase==0) {
      FlightRecord old;
      if(!io.read(nextAddress,&old,128) || !erasedRecord(old)){status=FAULT;return;}
      phase=1;return;
    }
    if(phase==1) {
      if(!io.program(nextAddress,&queue[0],127)){status=FAULT;return;}
      phase=2;return;
    }
    if(phase==2) {
      FlightRecord r;
      if(!io.read(nextAddress,&r,128) || !sameRecordBody(r,queue[0])){status=FAULT;return;}
      phase=3;return;
    }
    if(phase==3) {
      const uint8_t commit=0xA5;
      if(!io.program(nextAddress+127,&commit,1)){status=FAULT;return;}
      phase=4;return;
    }
    FlightRecord r;
    if(!io.read(nextAddress,&r,128) || !validRecord(r)){status=FAULT;return;}
    committed=r.sequence; nextAddress+=128;
    for(uint8_t i=0;i+1<count;++i)queue[i]=queue[i+1];
    --count; phase=0;
    if(nextAddress>=io.size())status=FULL;
  }
private:
  FlightRecord scanBuffer[RECOVERY_RECORDS]={};
  FlightRecord queue[8]={};
  uint8_t count=0,phase=0;
  uint32_t scanAddress=0,nextAddress=0,busySince=0;
};

struct MissionClock {
  bool launched=false,complete=false,armed=false;
  TimeQuality quality=TIME_UNSTARTED;
  uint32_t anchorMs=0,launchRtc=0;
  uint64_t elapsed=0;
  constexpr void tick(uint32_t now) {
    if(launched && !complete && quality!=TIME_UNCERTAIN)elapsed+=uint32_t(now-anchorMs);
    anchorMs=now;
  }
  constexpr void launch(uint32_t now,uint32_t trustedRtc) {
    if(launched)return;
    launched=true;elapsed=0;anchorMs=now;launchRtc=trustedRtc;quality=TIME_MONOTONIC;
  }
  constexpr void restore(const FlightRecord& r,uint32_t now,uint32_t trustedRtc) {
    launched=r.flags&RF_LAUNCHED;complete=r.flags&RF_COMPLETE;armed=false;
    launchRtc=r.launchRtc;anchorMs=now;elapsed=r.elapsed;quality=(TimeQuality)r.quality;
    if(!launched){elapsed=0;quality=TIME_UNSTARTED;return;}
    if(complete)return;
    if(quality!=TIME_UNCERTAIN && elapsed!=UNKNOWN_ELAPSED && (r.flags&RF_RTC_TRUSTED) &&
       trustedRtc>=r.rtcUnix && r.rtcUnix && trustedRtc-r.rtcUnix<=86400UL) {
      elapsed+=uint64_t(trustedRtc-r.rtcUnix)*1000;quality=TIME_RTC_RECOVERED;
    } else { quality=TIME_UNCERTAIN;elapsed=UNKNOWN_ELAPSED; }
  }
};
