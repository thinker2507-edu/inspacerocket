#pragma once
#include <SD.h>
#include "FlightCsv.h"

// Teensy 4.1 onboard SDIO slot, NOT an SPI SD module. The external NOR
// journal remains the recovery store; no routine samples fall back to NOR.
class SdFlightLog {
public:
  enum Status : uint8_t { MISSING, OPENING, READY, FAILED, CLOSED };
  Status status=MISSING;
  uint32_t dropped=0, written=0, slowOperations=0;
  char filename[13]={};
  void requestClose(){closing=true;}
  bool stopped() const {return status==MISSING || status==FAILED || status==CLOSED;}
  void begin(char vehicle) {
    prefix=vehicle;
    status=SD.begin(BUILTIN_SDCARD)?OPENING:MISSING;
    report();
  }
  bool append(FlightRecord r) {
    if(closing || (status!=READY && status!=OPENING)){++dropped;return false;}
    const bool critical=r.type!=LOG_SAMPLE;
    if(!critical && count>=12){++dropped;return false;}
    if(count==16){
      uint8_t i=0;while(i<count && queue[i].type!=LOG_SAMPLE)++i;
      if(i==count){++dropped;return false;}
      for(;i+1<count;++i)queue[i]=queue[i+1];
      --count;++dropped;
    }
    // This sequence is local to this boot's SD file, independent of LoRa and
    // NOR journal sequences. The queued snapshot is formatted as a CSV row.
    r.magic=RECORD_MAGIC;r.version=2;r.metadata.length=sizeof(r);
    r.sequence=nextSequence++;r.dropped=dropped;
    r.crc=recordCrc(r);r.commit=0xA5;
    queue[count++]=r;return true;
  }
  void service(uint32_t now) {
    if(status==OPENING){
      // One filename check per call; never scan or append an old flight file.
      if(fileIndex>=1000){fail();return;}
      snprintf(filename,sizeof(filename),"%c%07lu.CSV",prefix,(unsigned long)fileIndex++);
      if(SD.exists(filename))return;
      if(!file.open(filename,O_WRONLY|O_CREAT|O_EXCL)){fail();return;}
      headerPending=true;status=READY;lastSync=now;report();return;
    }
    if(status!=READY)return;
    if(file.isBusy()){
      if(!busy){busy=true;busySince=now;}
      else if(uint32_t(now-busySince)>250)fail();
      return;
    }
    busy=false;
    uint32_t started=micros();
    // Only one filesystem operation per call. sync() is separate from write().
    if(dirty && (urgentSync || (closing && !count) || uint32_t(now-lastSync)>=1000)){
      if(!file.sync()){fail();return;}
      dirty=false;urgentSync=false;lastSync=now;
    } else if(headerPending){
      size_t n=strlen(FLIGHT_CSV_HEADER);
      if(file.write(FLIGHT_CSV_HEADER,n)!=n || file.getWriteError()){fail();return;}
      headerPending=false;dirty=true;
    } else if(count){
      char row[1024];int n=formatFlightCsv(row,sizeof(row),queue[0],prefix);
      if(n<=0 || file.write(row,n)!=size_t(n) || file.getWriteError()){
        fail();return;
      }
      urgentSync=queue[0].type!=LOG_SAMPLE;dirty=true;++written;
      for(uint8_t i=0;i+1<count;++i)queue[i]=queue[i+1];
      --count;
    } else if(closing && !dirty){
      if(!file.close()){fail();return;}
      status=CLOSED;report();
    }
    // SD filesystem/card calls are synchronous and can occasionally stall.
    // This measures those stalls; it does not pretend to impose a hard deadline.
    if(uint32_t(micros()-started)>20000)++slowOperations;
  }
private:
  FsFile file;
  FlightRecord queue[16]={};
  uint8_t count=0;
  char prefix='R';
  uint32_t fileIndex=0,nextSequence=1,lastSync=0,busySince=0;
  bool dirty=false,urgentSync=false,busy=false,headerPending=false,closing=false;
  void report(){
    Serial.printf("# SD status=%u file=%s dropped=%lu\n",unsigned(status),filename,(unsigned long)dropped);
  }
  void fail(){status=FAILED;dropped+=count;count=0;report();}
};
