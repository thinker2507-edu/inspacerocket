#pragma once
#include <stdio.h>
#include "DescentGuidance.h"

// Monitor only: the reference assumes two positional pinch servos, whereas
// current PWM4 is continuous rotation and no left/right linkage is confirmed.
// No Servo objects, GPIO writes, deployment authorization or command retries.
class DescentMonitor {
public:
  enum Status : uint8_t { PAD_ACQUIRE, WAIT_FLIGHT, WAIT_CANOPY, MONITORING,
                          SENSOR_HOLD, RECOVERY_HOLD, LANDED };
  RocketDescent::DescentGuidance guidance;
  Status state=PAD_ACQUIRE;
  bool padReady=false, freshFix=false, freshImu=false;
  uint16_t padSamples=0;
  uint32_t rejectedFixes=0;

  constexpr void observeFix(double lat,double lon,uint8_t sats,uint32_t stamp,
                  bool velocityValid,float east,float north) {
    if(!isfinite(lat) || !isfinite(lon) || fabs(lat)>90 || fabs(lon)>180 ||
       sats<RocketDescent::MIN_SATS ||
       (velocityValid && (!isfinite(east) || !isfinite(north)))) {++rejectedFixes;return;}
    fix.lat=lat;fix.lon=lon;fix.sats=sats;fix.hasVel=velocityValid;
    fix.vE=east;fix.vN=north;fixMs=stamp;haveFix=pendingFix=true;
  }

  constexpr void service(uint32_t now,uint32_t missionId,uint32_t calibrationGeneration,
               uint8_t phase,bool launched,bool recovered,bool ready,
               bool baroValid,float relativeHeight,bool gyroValid,
               float gx,float gy,float gz,float gravityX,float gravityY,float gravityZ) {
    if(!initialized || missionId!=mission || calibrationGeneration!=calibration) {
      // An explicit new mission/calibration starts a fresh target and estimator.
      guidance=RocketDescent::DescentGuidance();
      mission=missionId;calibration=calibrationGeneration;initialized=true;
      padReady=false;padSamples=0;padLat=padLon=0;apogeeSeen=started=false;
      epochMs=lastMs=now;state=PAD_ACQUIRE;
    }
    const uint32_t elapsed=uint32_t(now-lastMs);lastMs=now;
    freshFix=haveFix && uint32_t(now-fixMs)<=RocketDescent::FIX_MAX_AGE_MS;
    const float norm=sqrtf(gravityX*gravityX+gravityY*gravityY+gravityZ*gravityZ);
    const float yaw=norm>0 ? (gx*gravityX+gy*gravityY+gz*gravityZ)/norm : NAN;
    freshImu=gyroValid && isfinite(yaw) && isfinite(norm) && norm>=8.5f && norm<=10.8f;
    const bool newFix=pendingFix && freshFix;pendingFix=false;
    if(phase==5){state=LANDED;return;}
    if(recovered || !ready){state=RECOVERY_HOLD;return;}
    if(phase==0 && !launched) {
      if(newFix && baroValid && freshImu && padSamples<5000){
        ++padSamples;padLat+=(fix.lat-padLat)/padSamples;padLon+=(fix.lon-padLon)/padSamples;
        // Existing FSW altitude is already relative to the calibrated pad.
        guidance.setPad(padLat,padLon,0);padReady=true;
      }
      state=padReady?WAIT_FLIGHT:PAD_ACQUIRE;return;
    }
    if(phase==3 && !apogeeSeen){apogeeSeen=true;apogeeMs=now;}
    if(phase!=4 || !launched){state=WAIT_FLIGHT;return;}
    if(!apogeeSeen || uint32_t(now-apogeeMs)<RocketDescent::CANOPY_WAIT_MS){state=WAIT_CANOPY;return;}
    if(!padReady || !freshFix || !freshImu || !baroValid || !isfinite(relativeHeight) ||
       !elapsed || elapsed>100){state=SENSOR_HOLD;return;}
    const float t=uint32_t(now-epochMs)*0.001f;
    // After a data interruption, discard old wind/heading and reacquire drift.
    if(!started || state==SENSOR_HOLD){
      guidance=RocketDescent::DescentGuidance();guidance.setPad(padLat,padLon,0);
      guidance.start(t);started=true;
    }
    RocketDescent::Sensors sample={};sample.alt_m=relativeHeight;sample.fix=newFix;
    sample.lat=fix.lat;sample.lon=fix.lon;sample.sats=fix.sats;sample.hasVel=fix.hasVel;
    sample.vE=fix.vE;sample.vN=fix.vN;sample.gz_dps=yaw;
    // gx/gy/gz are already bias corrected by the existing sensor path.
    guidance.update(sample,t,elapsed*0.001f);
    if(!isfinite(guidance.cmdL) || !isfinite(guidance.cmdR) ||
       !isfinite(guidance.psi) || !isfinite(guidance.Wx) || !isfinite(guidance.Wy)){
      started=false;state=SENSOR_HOLD;return;
    }
    state=MONITORING;
  }

  void status(char* info,size_t capacity) const {
    snprintf(info,capacity,"GUIDE=MONITOR;S=%u;M=%u;PAD=%u;FIX=%u;IMU=%u;L=%.2f;R=%.2f;W=%.1f/%.1f;OUT=DISABLED",
      unsigned(state),unsigned(guidance.mode),padReady,freshFix,freshImu,
      guidance.cmdL,guidance.cmdR,guidance.Wx,guidance.Wy);
  }
private:
  RocketDescent::Sensors fix={};
  double padLat=0,padLon=0;
  bool initialized=false,haveFix=false,pendingFix=false,apogeeSeen=false,started=false;
  uint32_t mission=0,calibration=0,epochMs=0,lastMs=0,fixMs=0,apogeeMs=0;
};
