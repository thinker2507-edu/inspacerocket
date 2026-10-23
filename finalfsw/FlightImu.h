#pragma once
#include <Adafruit_BNO08x.h>
#include "ImuSample.h"

// Keep Adafruit's transport, reset handling and SH-2 decoder. The installed I2C
// HAL never assigns *t_us, and its single-event callback overwrites earlier
// reports in a bundle. Correct both locally without editing an installed library.
// SH-2/Adafruit support one active device; this project has one BNO085 per MCU.
class FlightImu : public Adafruit_BNO08x {
public:
  FlightImu():Adafruit_BNO08x(-1){}
  void clearReports(){reports.clear();}
  bool getSensorEvent(sh2_SensorValue_t* value){
    if(!reports.count)sh2_service(); // one bounded transport poll
    return reports.pop(*value);
  }
  uint32_t droppedReports()const{return reports.dropped;}
  uint32_t decodeErrors=0;
protected:
  bool _init(int32_t sensorId) override {
    clearReports();
    transportRead=_HAL.read;
    if(!transportRead || transportRead==timestampedRead)return false;
    _HAL.read=timestampedRead;
    if(!Adafruit_BNO08x::_init(sensorId))return false;
    return sh2_setSensorCallback(received,this)==SH2_OK;
  }
private:
  using ReadFunction=int(*)(sh2_Hal_t*,uint8_t*,unsigned,uint32_t*);
  inline static ReadFunction transportRead=nullptr;
  ImuQueue<sh2_SensorValue_t,32> reports;
  static int timestampedRead(sh2_Hal_t* hal,uint8_t* data,unsigned length,uint32_t* timeUs){
    *timeUs=millis()*1000UL; // same clock domain as Adafruit hal_getTimeUs()
    return transportRead(hal,data,length,timeUs);
  }
  static void received(void* cookie,sh2_SensorEvent_t* event){
    auto* self=static_cast<FlightImu*>(cookie);
    switch(event->reportId){
      case SH2_ACCELEROMETER:case SH2_GYROSCOPE_CALIBRATED:
      case SH2_GRAVITY:case SH2_ROTATION_VECTOR:break;
      default:return;
    }
    sh2_SensorValue_t value={};
    if(sh2_decodeSensorEvent(&value,event)!=SH2_OK){++self->decodeErrors;return;}
    self->reports.push(value);
  }
};
