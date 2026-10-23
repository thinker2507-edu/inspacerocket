#pragma once
#include <Wire.h>
#include <RTClib.h>
#include <math.h>

// Checked transfers: failed devices invalidate only their own cached readings.
inline bool auxRead(TwoWire& bus,uint8_t address,uint8_t reg,uint8_t* data,uint8_t n) {
  bus.beginTransmission(address); bus.write(reg);
  if(bus.endTransmission(false)!=0)return false;
  if(bus.requestFrom(address,n)!=n){while(bus.available())bus.read();return false;}
  for(uint8_t i=0;i<n;++i)data[i]=bus.read();
  return true;
}
inline bool auxWrite16(TwoWire& bus,uint8_t address,uint8_t reg,uint16_t value) {
  bus.beginTransmission(address);bus.write(reg);bus.write(uint8_t(value>>8));bus.write(uint8_t(value));
  return bus.endTransmission()==0;
}

// Same INA219 32 V / 2 A calibration and units as inacode.ino / Adafruit's
// setCalibration_32V_2A(): 0.1 ohm shunt, calibration 4096, config 0x399F.
// No new library dependency. One checked I2C transaction per service invocation.
struct InaMonitor {
  bool present=false,valid=false;
  float voltage=NAN,shunt=NAN,current=NAN,power=NAN;
  uint32_t lastGood=0,nextMs=0;
  uint8_t phase=0;
  uint16_t busRaw=0,shuntRaw=0,currentRaw=0;
  void service(TwoWire& bus,uint8_t address,uint32_t now) {
    if(valid && uint32_t(now-lastGood)>1000)valid=false;
    if(int32_t(now-nextMs)<0)return;
    bool ok=true;uint8_t b[2]={};
    if(phase==0)ok=auxWrite16(bus,address,5,4096);
    else if(phase==1)ok=auxWrite16(bus,address,0,0x399F);
    else if(phase==2)ok=auxWrite16(bus,address,5,4096); // survive INA reset
    else {
      const uint8_t regs[]={2,1,4,3};
      ok=auxRead(bus,address,regs[phase-3],b,2);
      uint16_t raw=(uint16_t(b[0])<<8)|b[1];
      if(phase==3){busRaw=raw;ok=ok && !(raw&1) && (raw&2);}
      if(phase==4)shuntRaw=raw;
      if(phase==5)currentRaw=raw;
      if(phase==6 && ok){
        voltage=(busRaw>>3)*0.004f;shunt=int16_t(shuntRaw)*0.01f;
        current=int16_t(currentRaw)*0.1f;power=raw*2.0f;
        valid=present=true;lastGood=now;
      }
    }
    if(!ok){valid=present=false;phase=0;nextMs=now+5000;return;}
    if(phase==1){present=true;phase=2;nextMs=now+100;return;}
    if(phase==6){phase=2;nextMs=now+100;return;}
    ++phase;nextMs=now+1;
  }
};

inline bool readDs3231(uint32_t& unixTime) {
  uint8_t status=0,t[7]={};
  if(!auxRead(Wire,0x68,0x0F,&status,1) || (status&0x80) ||
     !auxRead(Wire,0x68,0,t,7))return false; // oscillator-stop flag invalidates time
  auto bcd=[](uint8_t v)->uint8_t{return (v>>4)*10+(v&15);};
  // Support the DS3231's 12-hour mode as well as RTClib's usual 24-hour mode.
  uint8_t hour=(t[2]&0x40)?bcd(t[2]&0x1F)%12+((t[2]&0x20)?12:0):bcd(t[2]&0x3F);
  for(uint8_t i=0;i<7;++i)if((t[i]&15)>9)return false;
  if((t[5]&0x80) || bcd(t[0]&0x7F)>59 || bcd(t[1]&0x7F)>59 || hour>23)return false;
  DateTime d(2000+bcd(t[6]),bcd(t[5]&0x1F),bcd(t[4]&0x3F),hour,bcd(t[1]),bcd(t[0]&0x7F));
  if(!d.isValid() || d.year()<2024 || d.year()>2099)return false;
  unixTime=d.unixtime();return true;
}
