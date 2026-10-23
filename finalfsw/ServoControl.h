#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// ROCKET SERVO CONFIGURATION (microseconds).
// User confirmed PWM4 is a continuous-rotation MG995: pulses control speed,
// not angle. PWM5 retains its separate positional/manual pulse configuration.
// Nominal RC profile: 50 Hz, 1000..2000 us; 1500 us is PWM4 neutral and
// PWM5's existing nominal center target. Continuous PWM4 has no angle target.
// Reference and variant limitations: ../feature.md. These are NOMINAL command
// targets, not measured angles or mechanically verified deployment positions.
// Source labels pwm4=22 and PWM5=23 are retained. Confirm PCB continuity,
// linkage, supply and 3.3 V signal compatibility on the actual hardware.
// Manual commands work now; startup/deployment confirmation is independent per
// output and remains false until the mechanism's safe/deploy positions are known.
// Pins 22/23 use independent FlexPWM4 submodules 0/1 on Teensy 4.1.
struct ServoConfig {
  uint8_t pin;
  bool enabled;
  uint16_t frameHz, minUs, maxUs, safeUs, deployUs;
  bool startupSafeConfirmed=false, deploymentConfirmed=false;
  uint16_t test90Us=1500;
  bool continuous=false;
  uint16_t spinUs=0,reverseUs=0;
  constexpr uint32_t duty(uint32_t us) const {
    return uint32_t((uint64_t(us)*frameHz*65536ULL+500000ULL)/1000000ULL);
  }
  constexpr bool valid() const {
    return enabled && (pin==22 || pin==23) && frameHz && minUs && minUs<maxUs &&
      uint64_t(maxUs)*frameHz<1000000ULL && duty(minUs)>0 && duty(maxUs)<65536 &&
      safeUs>=minUs && safeUs<=maxUs && deployUs>=minUs && deployUs<=maxUs &&
      test90Us>=minUs && test90Us<=maxUs &&
      (!continuous || (spinUs>=minUs && spinUs<=maxUs && reverseUs>=minUs && reverseUs<=maxUs));
  }
};
// pin, enabled, Hz, minUs, maxUs, safeUs, deployUs,
// startupSafeConfirmed, deploymentConfirmed, test90Us, continuous, spinUs, reverseUs.
// PWM4 SPIN/REVERSE are bounded test pulses, not calibrated RPM/direction or a
// timed one-revolution operation. STOP uses safeUs (nominal 1500 us; trim on unit).
constexpr ServoConfig PWM4_SERVO={22,true,50,1000,2000,1500,1500,false,false,1500,true,1700,1300};
constexpr ServoConfig PWM5_SERVO={23,true,50,1000,2000,1500,1500,false,false,1500};
static_assert(PWM4_SERVO.pin!=PWM5_SERVO.pin,"Servo pins must be independent");
static_assert(!PWM4_SERVO.enabled || PWM4_SERVO.valid(),"Invalid PWM4 configuration");
static_assert(!PWM5_SERVO.enabled || PWM5_SERVO.valid(),"Invalid PWM5 configuration");

struct ServoRequest {
  enum Result { ACCEPT, BAD_ARGUMENT, NOT_CONFIGURED, OUT_OF_RANGE };
  Result result;
  uint32_t us;
};
constexpr bool servoEquals(const char* a,const char* b){
  while(*a && *a==*b){++a;++b;}return *a==*b;
}
constexpr ServoRequest servoRequest(const ServoConfig& config,const char* arg){
  bool stop=config.continuous && servoEquals(arg,"STOP");
  bool spin=config.continuous && servoEquals(arg,"SPIN");
  bool reverse=config.continuous && servoEquals(arg,"REVERSE");
  bool safe=servoEquals(arg,"SAFE") || stop,test90=servoEquals(arg,"TEST90");
  uint32_t us=0;
  if(!safe && !test90 && !spin && !reverse){
    if(!*arg)return {ServoRequest::BAD_ARGUMENT,0};
    for(const char* p=arg;*p;++p){
      if(*p<'0' || *p>'9' || us>(UINT32_MAX-uint32_t(*p-'0'))/10U)
        return {ServoRequest::BAD_ARGUMENT,0};
      us=us*10U+uint32_t(*p-'0');
    }
  }
  if(!config.valid())return {ServoRequest::NOT_CONFIGURED,0};
  if(safe)us=config.safeUs;
  if(test90)us=config.test90Us;
  if(spin)us=config.spinUs;
  if(reverse)us=config.reverseUs;
  if(us<config.minUs || us>config.maxUs)return {ServoRequest::OUT_OF_RANGE,us};
  return {ServoRequest::ACCEPT,us};
}

// Manual wireless commands acknowledge acceptance before energizing the output.
// One radio command is already serialized by the existing ACK/TX dispatcher.
// This is independent of the descent intent and never writes the flight journal.
struct ServoRadioCommand {
  bool pending=false,ackTransmitting=false,resultPending=false,ackFailed=false;
  uint32_t sequence=0;
  uint8_t channel=0;
  uint16_t pulseUs=0;
  constexpr void queue(uint8_t output,uint16_t pulse,uint32_t seq){
    channel=output;pulseUs=pulse;sequence=seq;
    pending=true;ackTransmitting=resultPending=ackFailed=false;
  }
  constexpr void ackStarted(uint32_t seq,bool started){
    if(!pending || seq!=sequence || ackTransmitting)return;
    if(started)ackTransmitting=true;
    else {pending=false;ackFailed=resultPending=true;}
  }
  constexpr bool ready(bool txBusy) const {return pending && ackTransmitting && !txBusy;}
  constexpr void complete(){pending=ackTransmitting=false;resultPending=true;}
};

// IO supplies low(pin) and pulse(config,us). No sensors, mission clock or flash
// writes occur here. Tests inject an IO recorder; production uses Teensy PWM.
template<class IO> class ServoControl {
public:
  enum AutoStatus : uint8_t { IDLE, PERSISTING, COMMANDED, UNCONFIGURED,
                             STORAGE_FAILED, RECOVERY_INHIBITED };
  const ServoConfig config[2];
  IO& io;
  uint16_t commandedUs[2]={0,0}; // 0 = no pulses commanded this boot
  bool consumed=false, commanded=false, inhibited=false;
  AutoStatus automatic=IDLE;
  uint32_t token=0,queuedMs=0;
  constexpr ServoControl(IO& output,ServoConfig a,ServoConfig b):config{a,b},io(output){}
  constexpr void begin(){io.low(config[0].pin);io.low(config[1].pin);}
  constexpr void safeStartup(bool recoveredActiveOrAmbiguous){
    if(recoveredActiveOrAmbiguous)return; // never retract a recovered mechanism
    for(uint8_t i=0;i<2;++i)if(config[i].valid() && config[i].startupSafeConfirmed)write(i,config[i].safeUs);
  }
  constexpr void newMission(){consumed=commanded=inhibited=false;automatic=IDLE;token=0;}
  constexpr void restore(bool intent,bool priorCommand,bool ambiguous){
    consumed=intent;commanded=priorCommand;inhibited=intent || ambiguous;
    if(inhibited)automatic=RECOVERY_INHIBITED;
  }
  constexpr bool reserve(uint8_t previous,uint8_t next,bool launched,bool ready){
    if(previous!=3 || next!=4 || !launched || !ready || consumed || inhibited)return false;
    consumed=true;automatic=PERSISTING;return true;
  }
  constexpr void queued(uint32_t value,uint32_t now){
    token=value;queuedMs=now;if(!token)automatic=STORAGE_FAILED;
  }
  // At most one automatic command per channel after durable intent. A reset
  // between commit and pulses skips the action; recovery never replays it.
  constexpr bool service(uint32_t committed,bool healthy,uint32_t now){
    if(automatic!=PERSISTING || !token)return false;
    if(committed>=token){
      token=0;
      bool sent=false;
      for(uint8_t i=0;i<2;++i)if(config[i].valid() && config[i].deploymentConfirmed){write(i,config[i].deployUs);sent=true;}
      commanded=sent;automatic=sent?COMMANDED:UNCONFIGURED;return true;
    }
    if(!healthy || uint32_t(now-queuedMs)>5000){token=0;automatic=STORAGE_FAILED;}
    return false;
  }
  constexpr ServoRequest manual(uint8_t i,const char* arg){
    const ServoRequest request=servoRequest(config[i],arg);
    if(request.result==ServoRequest::ACCEPT)write(i,uint16_t(request.us));
    return request;
  }
  const char* test(uint8_t i,const char* arg,char* info,size_t capacity){
    const ServoRequest request=manual(i,arg);
    if(request.result==ServoRequest::BAD_ARGUMENT){
      snprintf(info,capacity,"%s",config[i].continuous?"ARG_SPIN_REVERSE_STOP_SAFE_TEST90_OR_US":"ARG_SAFE_TEST90_OR_US");return "ERR";
    }
    if(request.result==ServoRequest::NOT_CONFIGURED){snprintf(info,capacity,"PWM%u_UNCONFIGURED",i+4);return "REJECT";}
    if(request.result==ServoRequest::OUT_OF_RANGE){
      snprintf(info,capacity,"PWM%u_RANGE_%u_%u_US",i+4,config[i].minUs,config[i].maxUs);return "ERR";
    }
    snprintf(info,capacity,"PWM%u_CMD_US=%u;FEEDBACK=NONE",i+4,unsigned(request.us));return "OK";
  }
  void status(char* info,size_t capacity) const {
    snprintf(info,capacity,"P4=%u;U4=%u;D4=%u;P5=%u;U5=%u;D5=%u;AUTO=%u;INTENT=%u;SENT=%u;C4=%u;C5=%u;FEEDBACK=NONE",
      config[0].valid(),commandedUs[0],config[0].deploymentConfirmed,config[1].valid(),commandedUs[1],config[1].deploymentConfirmed,
      unsigned(automatic),consumed,commanded,config[0].continuous,config[1].continuous);
  }
private:
  constexpr void write(uint8_t i,uint16_t us){io.pulse(config[i],us);commandedUs[i]=us;}
};
