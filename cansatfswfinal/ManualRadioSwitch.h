#pragma once
#include <stdint.h>
#include <stddef.h>
#include "CommandValidation.h"

// Only the two operator-specified channels. All other LoRa parameters stay put.
enum class RadioChannel : uint8_t { MAIN=0, BACKUP=1, INVALID=255 };
constexpr RadioChannel radioChannel(const char* s){
  if(!s)return RadioChannel::INVALID;
  if(s[0]=='M' && s[1]=='A' && s[2]=='I' && s[3]=='N' && !s[4])return RadioChannel::MAIN;
  if(s[0]=='B' && s[1]=='A' && s[2]=='C' && s[3]=='K' && s[4]=='U' && s[5]=='P' && !s[6])return RadioChannel::BACKUP;
  return RadioChannel::INVALID;
}
constexpr const char* radioChannelName(RadioChannel c){return c==RadioChannel::BACKUP?"BACKUP":"MAIN";}
constexpr float radioChannelMHz(RadioChannel c){return c==RadioChannel::BACKUP?867.25f:865.50f;}
// Permit the single separator in a local receiver command, never whitespace
// inside an authenticated LoRa envelope.
constexpr bool receiverRadioSeparator(const char* prefix,size_t used,char c){
  if(c!=' ' || used!=6)return false;
  const char* local="@RADIO";
  for(size_t i=0;i<6;++i)if(prefix[i]!=local[i])return false;
  return true;
}
struct RadioSpan {size_t start=0,length=0;};
constexpr bool radioSpanIs(const char* s,RadioSpan span,const char* text){
  size_t n=0;while(text[n])++n;
  if(n!=span.length)return false;
  for(size_t i=0;i<n;++i)if(s[span.start+i]!=text[i])return false;
  return true;
}
constexpr bool radioSpanU32(const char* s,RadioSpan span,uint32_t& out){
  if(!span.length)return false;
  out=0;
  for(size_t i=0;i<span.length;++i){char c=s[span.start+i];
    if(c<'0' || c>'9' || out>(UINT32_MAX-uint32_t(c-'0'))/10U)return false;
    out=out*10U+uint32_t(c-'0');
  }return true;
}
constexpr unsigned radioFields(const char* s,size_t length,RadioSpan* fields,unsigned capacity){
  if(length<4)return 0;
  size_t start=1;unsigned count=0;
  for(size_t i=1;i<=length-3;++i)if(s[i]==',' || i==length-3){
    if(count>=capacity || i==start)return 0;
    fields[count++]={start,i-start};start=i+1;
  }return count;
}
struct RadioSwitchRequest {char vehicle=0;uint32_t sequence=0;RadioChannel channel=RadioChannel::MAIN;};
constexpr bool parseRadioSwitchRequest(const char* s,size_t n,RadioSwitchRequest& out){
  if(!validCommandFrame(s,n))return false;
  RadioSpan f[6]={};if(radioFields(s,n,f,6)!=6 || !radioSpanIs(s,f[4],"RADIO"))return false;
  uint32_t key=0,sequence=0;
  if(!radioSpanU32(s,f[2],key) || key!=4821 || !radioSpanU32(s,f[3],sequence))return false;
  char vehicle=radioSpanIs(s,f[1],"23:R")?'R':radioSpanIs(s,f[1],"23:C")?'C':0;
  RadioChannel c=radioSpanIs(s,f[5],"MAIN")?RadioChannel::MAIN:
                 radioSpanIs(s,f[5],"BACKUP")?RadioChannel::BACKUP:RadioChannel::INVALID;
  if(!vehicle || c==RadioChannel::INVALID)return false;
  out={vehicle,sequence,c};return true;
}
constexpr bool radioReplyMatches(const char* s,size_t n,const RadioSwitchRequest& request){
  if(fleetDownlinkVehicle(s,n,23)!=request.vehicle)return false;
  RadioSpan f[5]={};uint32_t sequence=0;
  return radioFields(s,n,f,5)==5 && radioSpanIs(s,f[0],"ACK") &&
    radioSpanU32(s,f[2],sequence) && sequence==request.sequence;
}
constexpr bool acceptedRadioSwitch(const char* s,size_t n,const RadioSwitchRequest& request){
  if(fleetDownlinkVehicle(s,n,23)!=request.vehicle)return false;
  RadioSpan f[5]={};uint32_t sequence=0;
  return radioFields(s,n,f,5)==5 && radioSpanIs(s,f[0],"ACK") &&
    radioSpanU32(s,f[2],sequence) && sequence==request.sequence &&
    radioSpanIs(s,f[3],"OK") && radioSpanIs(s,f[4],request.channel==RadioChannel::BACKUP?
      "RADIO_SWITCH_PENDING=BACKUP":"RADIO_SWITCH_PENDING=MAIN");
}
// No link-loss hopping or retry. A manual request waits for its acceptance TX,
// then a short receiver retune window. A lost over-air ACK requires manual GROUND
// recovery; successful local TX is deliberately not called confirmed reception.
struct ManualRadioSwitch {
  RadioChannel current=RadioChannel::MAIN,target=RadioChannel::MAIN;
  bool pending=false,fromUsb=false,ackStarted=false,ackFailed=false,waiting=false;
  uint32_t sequence=0,queuedMs=0,settleMs=0;
  constexpr void queue(RadioChannel channel,uint32_t seq,bool usb,uint32_t now){
    target=channel;sequence=seq;fromUsb=usb;queuedMs=now;
    pending=true;ackStarted=usb;ackFailed=waiting=false;
  }
  constexpr void acceptanceTx(uint32_t seq,bool started){
    if(!pending || fromUsb || seq!=sequence || ackStarted)return;
    ackStarted=started;ackFailed=!started;
  }
  constexpr bool ready(uint32_t now,bool busy){
    if(!pending || busy)return false;
    if(ackFailed || uint32_t(now-queuedMs)>5000)return true;
    if(!ackStarted)return false;
    if(!waiting){waiting=true;settleMs=now;return false;}
    return uint32_t(now-settleMs)>=250;
  }
};