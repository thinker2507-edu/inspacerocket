#pragma once
#include <stdint.h>
#include <stddef.h>

// Receive validity and calibration accuracy are distinct. Low-accuracy numeric
// data can be displayed, but reliable() remains required by fusion/calibration.
struct ImuSample {
  uint64_t stamp=0;
  uint32_t sampleMs=0,lastGoodArrivalMs=0,rejected=0;
  uint8_t sequence=0,accuracy=0;
  bool seen=false,valid=false;
  constexpr bool accept(uint64_t timestamp,uint8_t seq,uint8_t status,uint32_t now,bool finite){
    if(seen && timestamp==stamp && seq==sequence)return false; // no freshness refresh
    seen=true;stamp=timestamp;sequence=seq;accuracy=status&3;
    int32_t age=int32_t(now*1000UL-uint32_t(timestamp));
    valid=finite && age>=-2000 && age<=100000;
    if(!valid){++rejected;return false;}
    sampleMs=now-(age>0?uint32_t(age)/1000:0);
    lastGoodArrivalMs=now;return true;
  }
  constexpr bool fresh(uint32_t now)const{return valid && uint32_t(now-sampleMs)<=100;}
  constexpr bool reliable(uint32_t now)const{return fresh(now) && accuracy>=1;}
  constexpr uint32_t ageForStatus(uint32_t now)const{
    uint32_t age=uint32_t(now-sampleMs);return !valid || age>9999?9999:age;
  }
};

// Every decoded report in an SHTP bundle must be delivered, not just the last.
template<class T,size_t N> struct ImuQueue {
  T values[N]={};size_t head=0,count=0;
  uint32_t dropped=0;
  constexpr void clear(){head=count=0;}
  constexpr void push(const T& value){
    if(count==N){head=(head+1)%N;--count;++dropped;} // keep newest data on overflow
    values[(head+count)%N]=value;++count;
  }
  constexpr bool pop(T& value){
    if(!count)return false;
    value=values[head];head=(head+1)%N;--count;return true;
  }
};
