/*
 * ROCKET FLIGHT SOFTWARE - Teensy 4.1   (telemetry downlink + command uplink)
 * Sensors : BMP581 (baro), BNO085 (IMU), MAX-M10S (GNSS), SX1262 (LoRa), DS3231 (RTC)
 * Other   : RunCam on Serial5 (TX5 = pin 20, RX5 = pin 21)
 *
 * ---------------- Pinout ----------------
 *  LoRa SX1262   SPI: MOSI 11, MISO 12, SCK 13, NSS 2, RESET 6, BUSY 4, RXEN 1, TXEN 0
 *                DIO1/DIO2 not routed -> TX completion timed from airtime, RX is polled.
 *  W25Q64JV NOR flash CS = 10, shares SPI 11/12/13 with radio (separate NSS 2)
 *  BMP581/BNO085 Wire2 : SDA2 = 25, SCL2 = 24
 *  MAX-M10S      Serial7: GPS TX -> pin 28 (RX7), GPS RX <- pin 29 (TX7)
 *  DS3231 RTC    Wire  : SDA0 = 18, SCL0 = 19   (EEPROM 0x50 shares this bus, disabled)
 *  RunCam        Serial5: TX5 = 20, RX5 = 21
 *  INA219 (opt.) Wire1 : SDA1 = 17, SCL1 = 16
 *
 * ---------------- Telemetry packet (unchanged, 16 fields) ----------------
 *  team:vehicle,timestamp,count,alt,pressure,temp,volt,time,lat,lon,gpsAlt,sats,
 *  ax;ay;az,gx;gy;gz,stateNumber,source(1 = rocket)
 *  States: 0 PAD  1 POWERED_ASCENT  2 COAST  3 APOGEE  4 PARACHUTE_DESCENT  5 LANDED
 *
 * ---------------- Uplink (ground -> rocket) ----------------
 *  Command : $CMD,<team>:<R|C>,<key>,<seq>,<NAME>[,<arg>]*<XOR hex>
 *  Reply   : $ACK,<team>:<R|C>,<seq>,<status>,<info>*<XOR hex>
 *            status = OK | DONE | FAIL | REJECT | ERR      (info never contains commas)
 *  XOR = XOR of every character between '$' and '*' (NMEA style), 2 hex digits.
 *  The rocket listens whenever it is not transmitting. The ground station should send
 *  right after it receives a telemetry packet (about 50 ms later) and then wait for the ACK.
 *
 *  Commands (names are UPPERCASE)
 *   PING               link test, info = RSSI/SNR of the command as heard by the rocket
 *   STATUS[,CAMERA]    original status, or recording/landing/estimator diagnostics
 *   TELEM,1|0          telemetry on / off  (off only on PAD or LANDED; launch forces it on)
 *   RATE,<ms>          telemetry period 400..10000 ms
 *   CAM,1|0            experimental recording intents: BOTH send power-button 0x01
 *   CAMPWR             one RunCam power-button click (model-dependent effect)
 *   CAL,BARO|GYRO|ALL  3 s calibration, PAD only, rocket must be still. Second ACK = DONE/FAIL
 *   TIME,<unix>        set the RTC from ground UTC (PAD only)
 *   REBOOT             restart the Teensy (PAD only)
 *   ARM                initialize a new mission on stationary PAD/LANDED, recording inactive
 *
 * Libraries: RadioLib (6+), Adafruit BNO08x, SparkFun BMP581 Arduino Library,
 *            TinyGPSPlus, RTClib (Adafruit)
 */

// ---------------- Configuration ----------------
#define VEHICLE_ID        'R'      // explicit command/downlink destination
#define TEAM_ID           23       // <-- your team number

// LoRa (MUST match the receiver)
#define LORA_FREQ_MHZ             865.5       // or 867.25
#define LORA_BW_KHZ               125.0
#define LORA_SF                   7
#define LORA_CR                   5
#define LORA_SYNC_WORD            0x95
#define LORA_TX_POWER             14          // dBm
#define LORA_TCXO_VOLTAGE    1.8         // 0 = skip. Only for modules with a TCXO on DIO3
#define TELEMETRY_PERIOD_MS  500         // default period (changeable with RATE)

// Uplink
#define CMD_KEY              4821        // shared number, MUST match the ground station
#define CMD_ACK_DELAY_MS     60          // wait before replying so the ground can switch to RX
#define CMD_RX_POLL_MS       5
#define RATE_MIN_MS          400
#define RATE_MAX_MS          10000
#define CAL_DURATION_MS      3000

// RunCam (UART). Default = RunCam device protocol: 0xCC, 0x01, <action>, CRC8(DVB-S2)
// If your camera expects different bytes, edit camSend().
#define USE_RUNCAM           1
#define CAM_BAUD             115200
#include "RunCamControl.h"
#include "CommandValidation.h"
#include "TelemetryControl.h"
#include "RadioCoexistence.h"
#include "FlightEstimate.h"
#include "FlightCalibration.h"

// Pins (Teensy 4.1)
#define PIN_LORA_NSS         2
#define PIN_LORA_RST         6
#define PIN_LORA_BUSY        4
#define PIN_LORA_RXEN        1
#define PIN_LORA_TXEN        0
#define PIN_NOR_CS           10
#define PWM5                 23
#define pwm4                 22
#define PWM4                 pwm4 // preserve original spelling and pin assignment
#include "ServoControl.h"
static_assert(PWM4==PWM4_SERVO.pin && PWM5==PWM5_SERVO.pin,"Servo mapping changed");

// I2C
#define I2C_CLOCK_HZ         400000
#define BMP_ADDR_A           0x47
#define BMP_ADDR_B           0x46        // fallback (SDO to GND)
#define BNO_ADDR_A           0x4A
#define BNO_ADDR_B           0x4B

// RTC (DS3231 on Wire: SDA 18 / SCL 19)
#define USE_RTC              1
#define TIMESTAMP_FROM_RTC   0           // 0 = ms since boot, 1 = RTC ms since midnight

// EEPROM on Wire (same bus as RTC). Disabled for now - set to 1 to log every sent packet.
#define USE_EEPROM           0
#define EEPROM_ADDR          0x50
#define EEPROM_PAGE_SIZE     32
#define EEPROM_SIZE_BYTES    4096
#define EEPROM_LOG_START     16

// INA219 voltage monitor on Wire1; absence is nonfatal and retried at 5 s.
#define USE_INA219           1
#define INA_ADDR             0x40

// Flight logic (thresholds are starting values - tune with real flight data)
#define LOOP_PERIOD_MS       20          // 50 Hz
#define BOOT_CAL_MS          3000
#include "LaunchConfig.h"
#include "UnusedPins.h"
#include "ManualRadioSwitch.h"
ManualRadioSwitch radioSwitch;
#define BURNOUT_VEL_DROP_MPS 3.0f
#define BURNOUT_SAMPLES      5
#define APOGEE_DROP_M        1.5f
#define APOGEE_SAMPLES       5
#define APOGEE_HOLD_MS       1000
#define LANDED_ALT_M         5.0f
#define LANDED_VEL_MPS       1.0f
#define LANDED_TIME_MS       3000
#define ALT_ALPHA            0.25f
#define VEL_ALPHA            0.15f


// ---------------- Includes ----------------
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <RadioLib.h>
#include "FlightImu.h"
#include "SparkFun_BMP581_Arduino_Library.h"
#if __has_include(<TinyGPSPlus.h>)
  #include <TinyGPSPlus.h>
#else
  #include <TinyGPS++.h>
#endif
#if USE_RTC
  #include <RTClib.h>
#endif
#include "AuxSensors.h"
#include "FlashHardware.h"
#include "FlightJournal.h"
#include "SdFlightLog.h"
#include "ConsoleCommand.h"

// ---------------- Objects ----------------
SX1262 radio = new Module(PIN_LORA_NSS, RADIOLIB_NC, PIN_LORA_RST, PIN_LORA_BUSY);
BMP581 bmp;
FlightImu bno;
sh2_SensorValue_t imuVal;
TinyGPSPlus gps;
#if USE_RTC
RTC_DS3231 rtc;
#endif
InaMonitor ina219;
FlashHardware flashHardware;
FlightJournal<FlashHardware> journal(flashHardware);
SdFlightLog sdLog;
TeensyServoOutput servoOutput;
ServoControl<TeensyServoOutput> servos(servoOutput,PWM4_SERVO,PWM5_SERVO);
ServoRadioCommand servoRadio;
uint32_t lastRecoveryCheckpointMs=0;
MissionClock mission;
bool recoveryDone=false, recoveredMission=false, recoveryHold=false;
bool completionAttempted=false;
bool rtcValid=false, rtcTrusted=false, cameraRecoveryInhibit=false;
bool rtcAdvancing=false;
uint32_t rtcPollMs=0, rtcChangeMs=0, missionId=0, armToken=0;
uint32_t lastLogMs=0, lastSystemMs=0, landingRtc=0, resetCause=0;
uint32_t flashScanStartMs=0;
bool flashScanStarted=false,flashScanReported=false;
uint64_t landingElapsed=UNKNOWN_ELAPSED;
uint32_t cameraToken=0, cameraCommandSeq=0, cameraQueuedMs=0;
TelemetrySequence telemetrySequence;
RadioCoexistence coexistence;
CommandHistory commandHistory;
uint32_t sequenceToken=0, sequenceHighPending=0, migrationToken=0, migrationFloor=0;
bool sequenceKnown=false;
bool commandFromUsb=false,calReplyUsb=false,radioReady=false;
uint32_t usbCommandSequence=0;
uint32_t counterResetToken=0;
uint32_t padResetToken=0,padResetSequence=0,padResetQueuedMs=0;
bool padResetFromUsb=false,padResetResultPending=false;
char padResetStatus[10]="",padResetInfo[80]="";
FlightCalibration calibration, pendingCalibration;
bool baroReferenceReady=false, calibrationPending=false, calibrationFromCommand=false;
uint32_t calibrationToken=0, calibrationQueuedMs=0;
bool cameraQueued=false, cameraQueuedStart=false, cameraAutomatic=false;
bool cameraReplyUsb=false;
bool cameraResultPending=false;
char cameraResultStatus[10]="", cameraResultInfo[96]="";

void queueAck(uint32_t seq,const char* status,const char* info,bool toUsb=false);
// Explicit declarations also support the lightweight translation-unit check.
uint32_t logEvent(RecordType type,uint32_t commandSequence=UINT32_MAX);
bool queueCamera(bool start,bool automatic,uint32_t seq,uint32_t now);
void servicePersistence(uint32_t now);
void applyCalibration(const FlightCalibration& value,uint32_t now);
bool beginCalibrationWrite(FlightCalibration value,bool fromCommand,uint32_t now);
void serviceCalibrationStorage(uint32_t now);

// PAD is the startup state; explicit arming and committed storage gate launch detection.
// Fixed numeric IDs shared by telemetry, journal recovery and the GCS.
// Keep names in decision logic instead of scattering literal numbers.
enum FlightState { ST_PAD = 0, ST_ASCENT = 1, ST_COAST = 2,
                   ST_APOGEE = 3, ST_DESCENT = 4, ST_LANDED = 5 };
static_assert(ST_PAD==0 && ST_ASCENT==1 && ST_COAST==2 &&
              ST_APOGEE==3 && ST_DESCENT==4 && ST_LANDED==5,
              "Flight-state IDs must remain compatible with telemetry and storage");
const char* stateName[] = { "PAD", "POWERED_ASCENT", "COAST", "APOGEE",
                            "PARACHUTE_DESCENT", "LANDED" };
FlightState state = ST_PAD;

bool bmpOK = false, bnoOK = false, inaOK = false, rtcOK = false;

// Sensor values
float groundPressure = 101325.0f;
float altitude = 0, altPrev = 0, maxAlt = 0, velocity = 0, maxVel = 0;
float pressurePa = NAN, tempC = NAN;
float ax = 0, ay = 0, az = 0, accelG = 1.0f;
float gxRaw = 0, gyRaw = 0, gzRaw = 0;   // gyro as read, deg/s
float gxOff = 0, gyOff = 0, gzOff = 0;   // bias set by CAL,GYRO
float gx = 0, gy = 0, gz = 0;            // gyro after bias, deg/s (transmitted)
// RTC-final-style telemetry: latest decoded axes, independent of IMU state.
float telemAx=NAN,telemAy=NAN,telemAz=NAN;
float telemGxRaw=NAN,telemGyRaw=NAN,telemGzRaw=NAN;
float roll = 0, pitch = 0;               // local use only
VerticalEstimate vertical;
float gravityX = 0, gravityY = 0, gravityZ = 0, verticalAccel = 0;
uint32_t accelMs = 0, gravityMs = 0, gyroMs = 0;
uint64_t gyroStamp=0;
bool accelValid = false, gravityValid = false, gyroValid = false;
bool imuFresh = false, flightDataFresh = false, stationary = false;
ImuSample imuAccel,imuGyro,imuGravity;
uint8_t imuReportMask=0,imuConfigIndex=0;
uint32_t imuNextConfigMs=0,imuLastReconfigureMs=0,imuResetCount=0,imuConfigFailures=0;
uint64_t accelStamp = 0, gravityStamp = 0;
uint32_t padStableMs = 0;
bool padStable = false;
double padAccelSum = 0;
float padPressureMin = 0, padPressureMax = 0;
float landingBaroMin = 0, landingBaroMax = 0;
uint32_t baroReadMs = 0;
bool baroSeen = false, acceptedBaro = false;
uint16_t landingSamples = 0;
uint8_t fallbackLaunchCount = 0;

// GNSS values (hold last valid)
char     gnssTime[12] = "00:00:00";
bool     gnssTimeFresh = false;
double   gnssLat = 0, gnssLon = 0;
float    gnssAlt = 0;
uint8_t  gnssSats = 0;

// RTC timing
bool     rtcSynced    = false;
uint32_t rtcLastUnix  = 0;
uint32_t rtcSecOfDay  = 0;
uint32_t rtcAnchorMs  = 0;
char     rtcHms[12]   = "00:00:00";

// State machine helpers
uint8_t  launchCount = 0, apogeeCount = 0, burnoutCount = 0;
uint32_t stateEnterMs = 0, landedTimerStart = 0, lastLoop = 0;
bool     landedTimerRunning = false;
double   bootSum = 0;
uint32_t bootN = 0;

// LoRa TX/RX
bool     txBusy = false, radioListening = false;
uint32_t txStartMs = 0, txWaitMs = 0, lastTelemetryMs = 0, lastRxPollMs = 0, packetCount = 0;
uint32_t lastTxFinishedMs=0;
bool rxFrameActive=false;
uint32_t rxFrameSince=0,rxFrameWaitMs=0,bootAN=0;
char     txBuf[220];

// Uplink / command handling
bool     telemetryOn = false; // fresh PAD boot waits for authenticated TELEM,1
bool     telemetrySessionStarted = false; // no unsolicited SYS before first ON
uint32_t telemetryPeriodMs = TELEMETRY_PERIOD_MS;
bool     camRecording = false; // last recording request only; no camera state feedback
LandingStopTimer cameraLanding;
enum AutoStopStatus { AUTO_IDLE, AUTO_PENDING, AUTO_SENT_UNCONFIRMED, AUTO_FAILED, AUTO_MANUAL };
AutoStopStatus autoStopStatus = AUTO_IDLE;
bool camStartRequested = false, camStartTransmitted = false, camStartFwAck = false;
bool camStopTransmitted = false, camStopFwAck = false;
bool     rebootPending = false;
uint32_t rebootRequestedMs=0,rebootReadyMs=0;
bool rebootReady=false;

bool     ackPending = false;
uint32_t ackDueMs = 0;
uint32_t radioAckSequence=0;
char     ackBuf[200];
int      ackLen = 0;

bool     haveCmdSeq = false;
uint32_t lastCmdSeq = 0;
char     lastStatus[10] = "OK";
char     lastInfo[128]   = "";

// Calibration (runs without blocking, PAD only)
#define CAL_BARO 1
#define CAL_GYRO 2
#define CAL_ACCEL 4
uint8_t  calMask = 0;
uint32_t calStartMs = 0, calSeq = 0, calPN = 0, calGN = 0;
double   calP = 0, calGx = 0, calGy = 0, calGz = 0;
double calVertical=0;
uint32_t calAN=0;
uint64_t calLastGyroStamp=0,calLastAccelStamp=0;
float calPressureMin=0,calPressureMax=0;
bool     calReport = false;
char     calStatus[10] = "DONE";
char     calInfo[96]   = "";

uint8_t  gpsRxBuf[512];
uint32_t eepromLogAddr = EEPROM_LOG_START;

// ---------------- Helpers ----------------
float pressureToAltitude(float p, float p0) {
  return relativePressureAltitude(p,p0);
}

bool readBaro() {
  uint8_t status = 0;
  if (bmp.getInterruptStatus(&status) != BMP5_OK || !(status & BMP5_INT_ASSERTED_DRDY)) return false;
  bmp5_sensor_data d;
  if (bmp.getSensorData(&d) != BMP5_OK) return false;
  if (!isfinite(d.pressure) || d.pressure < 2000 || d.pressure > 120000 ||
      !isfinite(d.temperature) || d.temperature < -80 || d.temperature > 100) return false;
  pressurePa = d.pressure;
  tempC = d.temperature;
  return true;
}

// Configuration writes complete synchronously in SH-2; no reset/begin/delay in loop.
bool requestIMUReport(uint8_t index) {
  const sh2_SensorId_t ids[]={SH2_ACCELEROMETER,SH2_GYROSCOPE_CALIBRATED,SH2_ROTATION_VECTOR,SH2_GRAVITY};
  const uint32_t intervals[]={10000,20000,20000,20000};
  bool ok=bno.enableReport(ids[index],intervals[index]);
  if(ok)imuReportMask|=uint8_t(1U<<index);
  else {imuReportMask&=uint8_t(~(1U<<index));++imuConfigFailures;}
  return ok;
}

void enableIMUReports() {
  imuReportMask=0;
  for(uint8_t i=0;i<4;++i)requestIMUReport(i);
  imuLastReconfigureMs=millis();imuNextConfigMs=millis()+2000;imuConfigIndex=0;
}

void readIMU() {
  if(!bnoOK)return;
  if(bno.wasReset()){
    accelValid=gravityValid=gyroValid=false;
    accelStamp=gravityStamp=gyroStamp=0;
    imuAccel=ImuSample();imuGyro=ImuSample();imuGravity=ImuSample();
    bno.clearReports();++imuResetCount;
    imuReportMask=0;imuConfigIndex=0;imuNextConfigMs=millis();imuLastReconfigureMs=millis();
    Serial.println("# BNO085 reset: restoring reports");
  }
  uint8_t events=0;
  while(events++<32 && bno.getSensorEvent(&imuVal)){
    uint32_t receivedMs=millis();
    switch(imuVal.sensorId){
      case SH2_ACCELEROMETER: {
        float x=imuVal.un.accelerometer.x,y=imuVal.un.accelerometer.y,z=imuVal.un.accelerometer.z;
        telemAx=x;telemAy=y;telemAz=z;
        float norm=sqrtf(x*x+y*y+z*z)/9.80665f;
        if(imuAccel.accept(imuVal.timestamp,imuVal.sequence,imuVal.status,receivedMs,isfinite(norm) && norm<15.0f)){
          ax=x;ay=y;az=z;accelG=norm;accelMs=imuAccel.sampleMs;accelStamp=imuVal.timestamp;
        }
        break;
      }
      case SH2_GRAVITY: {
        float x=imuVal.un.gravity.x,y=imuVal.un.gravity.y,z=imuVal.un.gravity.z;
        float norm=sqrtf(x*x+y*y+z*z);
        if(imuGravity.accept(imuVal.timestamp,imuVal.sequence,imuVal.status,receivedMs,isfinite(norm) && norm>=8.5f && norm<=10.8f)){
          gravityX=x;gravityY=y;gravityZ=z;gravityMs=imuGravity.sampleMs;gravityStamp=imuVal.timestamp;
        }
        break;
      }
      case SH2_GYROSCOPE_CALIBRATED: {
        float x=imuVal.un.gyroscope.x*57.29578f,y=imuVal.un.gyroscope.y*57.29578f,z=imuVal.un.gyroscope.z*57.29578f;
        telemGxRaw=x;telemGyRaw=y;telemGzRaw=z;
        if(imuGyro.accept(imuVal.timestamp,imuVal.sequence,imuVal.status,receivedMs,
          isfinite(x-gxOff) && isfinite(y-gyOff) && isfinite(z-gzOff))){
          gxRaw=x;gyRaw=y;gzRaw=z;gx=x-gxOff;gy=y-gyOff;gz=z-gzOff;
          gyroMs=imuGyro.sampleMs;gyroStamp=imuVal.timestamp;
        }
        break;
      }
      case SH2_ROTATION_VECTOR: {
        float w=imuVal.un.rotationVector.real,x=imuVal.un.rotationVector.i,y=imuVal.un.rotationVector.j,z=imuVal.un.rotationVector.k;
        if(!isfinite(w) || !isfinite(x) || !isfinite(y) || !isfinite(z))break;
        roll=atan2f(2.0f*(w*x+y*z),1.0f-2.0f*(x*x+y*y))*57.29578f;
        float s=2.0f*(w*y-z*x);s=fmaxf(-1.0f,fminf(1.0f,s));pitch=asinf(s)*57.29578f;
        break;
      }
    }
  }
  uint32_t now=millis();
  // Flight-only validity: these flags never suppress the direct telemetry readings.
  accelValid=imuAccel.reliable(now);gyroValid=imuGyro.reliable(now);gravityValid=imuGravity.reliable(now);
  if(uint32_t(now-imuLastReconfigureMs)>=2000 &&
    (uint32_t(now-imuAccel.lastGoodArrivalMs)>1000 || uint32_t(now-imuGyro.lastGoodArrivalMs)>1000 ||
     uint32_t(now-imuGravity.lastGoodArrivalMs)>1000)){
    imuReportMask=0;imuConfigIndex=0;imuNextConfigMs=now;imuLastReconfigureMs=now;
  }
  if(imuReportMask!=15 && int32_t(now-imuNextConfigMs)>=0){
    for(uint8_t i=0;i<4;++i){
      uint8_t index=imuConfigIndex;imuConfigIndex=(imuConfigIndex+1)%4;
      if(imuReportMask&(1U<<index))continue;
      bool ok=requestIMUReport(index);imuNextConfigMs=millis()+(ok?1:2000);break;
    }
  }
}

void readGPS() {
  while (Serial7.available()) gps.encode(Serial7.read());
  if (gps.location.isValid()) {
    gnssLat = gps.location.lat();
    gnssLon = gps.location.lng();
  }
  if (gps.altitude.isValid())   gnssAlt  = (float)gps.altitude.meters();
  if (gps.satellites.isValid()) gnssSats = (uint8_t)gps.satellites.value();
  gnssTimeFresh = gps.time.isValid() && gps.time.age() < 2000;
  if (gnssTimeFresh) {
    snprintf(gnssTime, sizeof(gnssTime), "%02u:%02u:%02u",
             gps.time.hour(), gps.time.minute(), gps.time.second());
  }
}

float readBusVoltage() {
#if USE_INA219
  if (ina219.valid) return ina219.voltage;
#endif
  return NAN;   // absent/failed sensor is not a measured zero volts
}

bool inFlight() {
  return state == ST_ASCENT || state == ST_COAST || state == ST_APOGEE || state == ST_DESCENT;
}

uint8_t packetSequenceStatus() {
  if(!recoveryDone)return 0;
  if(!sequenceKnown)return 2; // legacy log needs an explicit safe counter baseline
  if(!telemetrySequence.next)return 4;
  if(telemetrySequence.available())return 1;
  if(!journal.usable() || journal.criticalLost)return 3;
  return 5; // reservation being committed; no numbered TX yet
}

// ---------------- RTC ----------------
void updateRTC() {
#if USE_RTC
  if (!rtcOK) return;
  uint32_t now=millis();
  if(uint32_t(now-rtcPollMs)<1000)return;
  rtcPollMs=now;
  uint32_t u=0;
  if(!readDs3231(u)){rtcValid=rtcTrusted=rtcAdvancing=false;return;}
  if(rtcLastUnix && (u<rtcLastUnix || u-rtcLastUnix>3))rtcTrusted=false;
  if (u != rtcLastUnix) {
    rtcAdvancing=rtcLastUnix && u>rtcLastUnix && u-rtcLastUnix<=3;
    rtcChangeMs=now;
    rtcLastUnix = u;
    rtcAnchorMs = now;
    DateTime t(u);
    rtcSecOfDay = (uint32_t)t.hour() * 3600UL + (uint32_t)t.minute() * 60UL + t.second();
    snprintf(rtcHms, sizeof(rtcHms), "%02u:%02u:%02u", t.hour(), t.minute(), t.second());
  }
  rtcValid=uint32_t(now-rtcChangeMs)<2500;
  if(!rtcValid)rtcTrusted=rtcAdvancing=false;
#endif
}

// Set the RTC to GNSS UTC once per boot, on the pad only (accuracy about +/- 1 s)
void syncRTCFromGNSS() {
#if USE_RTC
  if (!rtcOK || rtcSynced || !recoveryDone || mission.launched) return;
  if (state != ST_PAD) return;
  if (!gps.location.isValid() || !gps.time.isValid() || !gps.date.isValid()) return;
  if (gps.time.age() > 2000 || gps.date.age() > 2000) return;
  if (gps.date.year() < 2024 || gps.date.year()>2099) return;

  DateTime g(gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
  long diff = (long)rtcLastUnix - (long)g.unixtime();
  if (diff < 0) diff = -diff;
  if (diff > 1 || !rtcValid) {
    rtc.adjust(g);
    Serial.println("# RTC set from GNSS (UTC)");
  }
  rtcSynced = true;
  uint32_t checked=0;
  rtcValid=readDs3231(checked) && checked>=g.unixtime() && checked-g.unixtime()<=1;
  rtcTrusted=rtcValid;rtcLastUnix=checked;rtcChangeMs=millis();
  logEvent(LOG_CLOCK);
#endif
}

uint32_t timestampMs(uint32_t nowMs) {
#if USE_RTC && TIMESTAMP_FROM_RTC
  if (rtcValid && rtcLastUnix != 0) {
    uint32_t sub = nowMs - rtcAnchorMs;
    if (sub > 999) sub = 999;
    return rtcSecOfDay * 1000UL + sub;
  }
#endif
  return nowMs;
}

// ---------------- EEPROM (disabled by default) ----------------
#if USE_EEPROM
bool eepromWrite(uint16_t addr, const uint8_t* data, size_t len) {
  while (len > 0) {
    size_t pageLeft = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
    size_t chunk = (len < pageLeft) ? len : pageLeft;

    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    Wire.write(data, chunk);
    if (Wire.endTransmission() != 0) return false;

    uint32_t t0 = millis();
    while (millis() - t0 < 10) {
      Wire.beginTransmission(EEPROM_ADDR);
      if (Wire.endTransmission() == 0) break;
    }
    addr += chunk; data += chunk; len -= chunk;
  }
  return true;
}

void eepromLogPacket(const char* pkt, int n) {
  if (n <= 0 || n > 255) return;
  if (eepromLogAddr + 1 + n > EEPROM_SIZE_BYTES) return;
  uint8_t l = (uint8_t)n;
  if (!eepromWrite(eepromLogAddr, &l, 1)) return;
  if (!eepromWrite(eepromLogAddr + 1, (const uint8_t*)pkt, n)) return;
  eepromLogAddr += 1 + n;
}
#endif

uint8_t wireState() {
  return (uint8_t)state;
}

void setState(FlightState s) {
  if (state == ST_LANDED && s != ST_PAD) return; // terminal until intentional new mission
  const FlightState previous=state;
  state = s;
  stateEnterMs = millis();
  apogeeCount = 0;
  burnoutCount = 0;
  launchCount = 0;
  landedTimerRunning = false;
  if (s == ST_PAD) {
    vertical = VerticalEstimate();
    altitude = altPrev = velocity = maxAlt = maxVel = 0;
    bootSum = bootN = bootAN = 0; padAccelSum = 0; padStable = false;
    fallbackLaunchCount = 0; landingSamples = 0;
    cameraLanding.reset(); autoStopStatus = AUTO_IDLE;
    camRecording = camStartRequested = camStartTransmitted = false;
    camStartFwAck = camStopTransmitted = camStopFwAck = false;
  }
  if (s == ST_LANDED) {
    cameraLanding.confirm(stateEnterMs);
    if (!cameraLanding.attempted) autoStopStatus = camRecording ? AUTO_PENDING : AUTO_IDLE;
  }
  if (s == ST_ASCENT) {
    telemetryOn = true;
    mission.launch(stateEnterMs,rtcTrusted && rtcValid?rtcLastUnix:0);
  }
  if(s==ST_LANDED){landingRtc=rtcValid?rtcLastUnix:0;landingElapsed=mission.elapsed;}
  bool servoIntent=servos.reserve(previous,s,mission.launched,recoveryDone && !recoveryHold);
  if(recoveryDone && s!=ST_PAD){
    uint32_t stateToken=logEvent(s==ST_ASCENT?LOG_LAUNCH:LOG_STATE);
    if(servoIntent)servos.queued(stateToken,stateEnterMs);
  }
  Serial.print("# STATE -> ");
  Serial.println(stateName[s]);
}

// ---------------- RunCam ----------------
bool camSend(uint8_t action) {
#if USE_RUNCAM
  uint8_t p[4];
  if (!buildRunCamPacket(action, p)) return false;
  if (Serial5.availableForWrite() < (int)sizeof(p)) return false;
  if (Serial5.write(p, sizeof(p)) != sizeof(p)) return false;
  Serial.print("# CAM TX:");
  for (size_t i = 0; i < sizeof(p); i++) {
    Serial.print(' ');
    if (p[i] < 16) Serial.print('0');
    Serial.print(p[i], HEX);
  }
  Serial.println();
  return true; // UART queued bytes, not confirmation of physical camera state
#else
  return false;
#endif
}

bool camPowerButton() { return camSend(CAM_ACT_POWER); }
bool camStartRecording() { return camSend(CAM_ACT_REC_START); }
bool camStopRecording() { return camSend(CAM_ACT_REC_STOP); }

void serviceCamera(uint32_t now) {
  if(padResetToken)return;
  if(cameraQueued) {
    bool durable=cameraToken && journal.committed>=cameraToken;
    bool failed=!journal.usable() || uint32_t(now-cameraQueuedMs)>5000;
    if(!durable && !failed)return;
    bool sent=durable && (cameraQueuedStart?camStartRecording():camStopRecording());
    if(cameraQueuedStart){camStartTransmitted=sent;camRecording=sent;}
    else {camStopTransmitted=sent;if(sent)camRecording=false;autoStopStatus=sent?AUTO_SENT_UNCONFIRMED:AUTO_FAILED;}
    cameraQueued=false;
    logEvent(LOG_CAM_RESULT,cameraAutomatic?UINT32_MAX:cameraCommandSeq);
    if(!cameraAutomatic){
      snprintf(cameraResultStatus,sizeof(cameraResultStatus),"%s",sent?"DONE":"FAIL");
      snprintf(cameraResultInfo,sizeof(cameraResultInfo),"%s",sent?(cameraQueuedStart?"CAM_REC_START_SENT":"CAM_REC_STOP_SENT"):"CAM_NOT_SENT_STORAGE_OR_UART");
      cameraResultPending=true;
    }
  }
  if (cameraLanding.due(now)) {
    cameraLanding.attempted = true; // latch BEFORE UART attempt, even on failure
    if (!camRecording) { autoStopStatus = AUTO_IDLE; return; }
    if(cameraRecoveryInhibit || !queueCamera(false,true,0,now))autoStopStatus=AUTO_FAILED;
  }
}

// ---------------- Persistent flight records / mission recovery ----------------
FlightRecord flightSnapshot(RecordType type,uint32_t commandSequence) {
  FlightRecord r={};
  r.type=type;r.mission=missionId;r.bootMs=millis();r.state=state;r.quality=mission.quality;
  r.elapsed=mission.elapsed;r.launchRtc=mission.launchRtc;
  r.rtcUnix=rtcValid?rtcLastUnix:0;r.landingRtc=landingRtc;r.landingElapsed=landingElapsed;
  r.flags=(mission.launched?RF_LAUNCHED:0)|(mission.complete?RF_COMPLETE:0)|
    (mission.armed?RF_ARMED:0)|(rtcValid?RF_RTC_VALID:0)|(rtcTrusted&&rtcValid?RF_RTC_TRUSTED:0)|
    (ina219.valid?RF_INA_VALID:0)|(camRecording?RF_CAM_ACTIVE:0)|
    (camStartTransmitted?RF_START_SENT:0)|(cameraLanding.attempted?RF_STOP_INTENT:0)|
    (camStopTransmitted?RF_STOP_SENT:0)|(recoveredMission?RF_RECOVERED:0)|
    (cameraRecoveryInhibit?RF_INHIBIT_CAMERA:0)|((!journal.usable()||journal.criticalLost)?RF_STORAGE_FAULT:0)|
    (recoveryHold?RF_RECOVERY_HOLD:0)|
    (camStartRequested?RF_START_REQUESTED:0)|
    ((baroSeen && uint32_t(r.bootMs-baroReadMs)<=100)?RF_BARO_VALID:0)|
    (imuFresh?RF_IMU_VALID:0)|
    (servos.consumed?RF_SERVO_INTENT:0)|(servos.commanded?RF_SERVO_COMMANDED:0)|
    (!sequenceKnown?RF_COUNTER_UNKNOWN:0)|
    ((haveCmdSeq||commandSequence!=UINT32_MAX)?RF_CMD_SEQ:0);
  r.groundPressure=groundPressure;r.altitude=altitude;r.velocity=velocity;
  r.maxAltitude=maxAlt;r.maxVelocity=maxVel;r.ax=ax;r.ay=ay;r.az=az;
  r.voltage=readBusVoltage();r.shunt_mV=ina219.valid?ina219.shunt:NAN;
  r.current_mA=ina219.valid?ina219.current:NAN;r.power_mW=ina219.valid?ina219.power:NAN;
  r.commandSequence=commandSequence==UINT32_MAX?lastCmdSeq:commandSequence;
  r.resetCause=resetCause;
  r.metadata.length=sizeof(r);
  r.metadata.telemetryHigh=sequenceKnown?
    (sequenceHighPending>telemetrySequence.high?sequenceHighPending:telemetrySequence.high):0;
  r.metadata.calibrationGeneration=calibration.generation;
  return r;
}

// External flash stores only recovery checkpoints, not the flight sample log.
uint32_t persistCheckpoint(FlightRecord r) {
  if(counterResetToken || padResetToken)return 0; // do not append an old counter bound after a reset boundary
  sdLog.append(r);
  return journal.append(r);
}

uint32_t logEvent(RecordType type,uint32_t commandSequence) {
  FlightRecord r=flightSnapshot(type,commandSequence);
  if(type==LOG_SAMPLE)return sdLog.append(r)?1:0;
  return persistCheckpoint(r);
}

bool queueCamera(bool start,bool automatic,uint32_t seq,uint32_t now) {
  if(cameraQueued || !journal.usable())return false;
  // Commit the intent BEFORE the UART toggle. A reset in either interval is
  // ambiguous, so recovery inhibits automatic camera actions instead of replaying.
  FlightRecord intent=flightSnapshot(LOG_CAM_INTENT,automatic?UINT32_MAX:seq);
  intent.metadata.detail=start?1:2; // intent kind; not a camera operation byte
  cameraToken=persistCheckpoint(intent);
  if(!cameraToken)return false;
  cameraQueued=true;cameraQueuedStart=start;cameraAutomatic=automatic;
  cameraCommandSeq=seq;cameraQueuedMs=now;
  return true;
}

void servicePersistence(uint32_t now) {
  if(rebootPending)return; // drain existing checkpoints without creating new ones
  if(!recoveryDone) {
    if(journal.scanning())return;
    if(journal.haveRecord && (journal.recovered.flags&RF_RTC_TRUSTED) && rtcOK &&
       !rtcAdvancing && uint32_t(now-rtcAnchorMs)<3000)return;
    recoveryDone=true;
    // Every invalid slot could conceal one committed 64-number reservation.
    // Burn that much space, rather than reusing a possibly transmitted number.
    telemetrySequence.restore(recoverTelemetryUpperBound(journal.telemetryHigh,journal.damagedSlots));
    sequenceKnown=(!journal.haveLegacy && !journal.damagedSlots) || journal.telemetryHigh!=0;
    packetCount=telemetrySequence.high;
    if(journal.haveCalibration){
      const FlightRecord& c=journal.calibration;
      FlightCalibration saved;
      saved.generation=c.metadata.calibrationGeneration;saved.pressure=c.groundPressure;
      saved.gyroX=c.ax;saved.gyroY=c.ay;saved.gyroZ=c.az;saved.verticalBias=c.voltage;
      if(saved.valid())applyCalibration(saved,now);
    }
    if(journal.haveRecord) {
      const FlightRecord& r=journal.recovered;
      bool trusted=rtcValid && rtcAdvancing && (r.flags&RF_RTC_TRUSTED) && r.rtcUnix &&
                   rtcLastUnix>=r.rtcUnix && rtcLastUnix-r.rtcUnix<=86400UL;
      rtcTrusted=trusted;rtcSynced=trusted;
      mission.restore(r,now,trusted?rtcLastUnix:0);
      missionId=r.mission;recoveredMission=true;
      landingRtc=r.landingRtc;landingElapsed=r.landingElapsed;
      // A previously armed pad with no committed launch is ambiguous after reset.
      recoveryHold=(!mission.launched && (r.flags&RF_ARMED)) || journal.tailTorn ||
                   (r.flags&(RF_RECOVERY_HOLD|RF_STORAGE_FAULT));
      cameraRecoveryInhibit=true;cameraLanding.attempted=true;
      camStartRequested=r.flags&RF_START_REQUESTED;
      camRecording=(r.flags&RF_CAM_ACTIVE) || (camStartRequested && !(r.flags&RF_STOP_SENT)) ||
                   (r.type==LOG_CAM_INTENT && r.metadata.detail==1);
      camStartTransmitted=r.flags&RF_START_SENT;
      camStopTransmitted=r.flags&RF_STOP_SENT;
      if(camRecording)autoStopStatus=AUTO_FAILED; // explicitly report reset inhibition
      if(r.flags&RF_CMD_SEQ){
        haveCmdSeq=true;lastCmdSeq=r.commandSequence;
        snprintf(lastStatus,sizeof(lastStatus),"REJECT");
        snprintf(lastInfo,sizeof(lastInfo),"RECOVERED_SEQ_OUTCOME_UNKNOWN");
      }
      if(mission.launched) {
        state=r.state>=ST_ASCENT && r.state<=ST_LANDED?(FlightState)r.state:ST_ASCENT;
        stateEnterMs=now;telemetryOn=true;
        if(isfinite(r.groundPressure) && r.groundPressure>2000 && r.groundPressure<120000 &&
           isfinite(r.altitude) && fabsf(r.altitude)<20000 && isfinite(r.velocity) && fabsf(r.velocity)<400) {
          groundPressure=r.groundPressure;baroReferenceReady=true;vertical.reset(now,calibration.verticalBias);
          altitude=altPrev=vertical.height=vertical.baroHeight=r.altitude;
          velocity=vertical.speed=vertical.baroSpeed=r.velocity;
          maxAlt=r.maxAltitude;maxVel=r.maxVelocity;
        } else {vertical=VerticalEstimate();recoveryHold=true;}
        if(state==ST_LANDED){cameraLanding.confirm(now);autoStopStatus=camRecording?AUTO_FAILED:AUTO_IDLE;}
      }
    }
    if(journal.tailTorn || (journal.status!=FlightJournal<FlashHardware>::READY &&
                           journal.status!=FlightJournal<FlashHardware>::FULL)) {
      recoveryHold=true;mission.armed=false;cameraRecoveryInhibit=true;
      if(mission.launched && !mission.complete){mission.quality=TIME_UNCERTAIN;mission.elapsed=UNKNOWN_ELAPSED;}
    }
    bool servoAmbiguous=journal.damagedSlots || journal.tailTorn || recoveryHold ||
      (mission.launched && state>=ST_DESCENT);
    servos.restore(journal.haveRecord && (journal.recovered.flags&RF_SERVO_INTENT),
      journal.haveRecord && (journal.recovered.flags&RF_SERVO_COMMANDED),servoAmbiguous);
    servos.safeStartup(mission.launched || mission.armed || servoAmbiguous || servos.consumed);
    // Startup PAD is unarmed; recovery never authorizes a new mission.
    logEvent(LOG_RESET);logEvent(LOG_RECOVERY);
  }
  if(padResetToken){
    bool done=journal.committed>=padResetToken;
    bool failed=!journal.usable() || uint32_t(now-padResetQueuedMs)>5000;
    if(!done && !failed)return;
    if(done){
      mission=MissionClock();mission.anchorMs=now;++missionId;
      servos.newMission(); // explicit durable new mission; no servo movement
      completionAttempted=false;armToken=0;
      landingRtc=0;landingElapsed=UNKNOWN_ELAPSED;
      recoveredMission=recoveryHold=cameraRecoveryInhibit=false;
      setState(ST_PAD);baroReferenceReady=false;
      snprintf(padResetStatus,sizeof(padResetStatus),"DONE");
      snprintf(padResetInfo,sizeof(padResetInfo),"ST=0;PAD_RESET_DONE;UNARMED;MISSION=%lu",(unsigned long)missionId);
    } else {
      mission.armed=false;recoveryHold=true;
      journal.status=FlightJournal<FlashHardware>::FAULT; // ambiguous reset: freeze writes until boot recovery
      snprintf(padResetStatus,sizeof(padResetStatus),"FAIL");
      snprintf(padResetInfo,sizeof(padResetInfo),"PAD_RESET_STORAGE_FAILED_REBOOT_TO_RESOLVE");
    }
    padResetToken=0;
    padResetResultPending=true;
    return;
  }
  if(counterResetToken)return;
  serviceCalibrationStorage(now);
  if(migrationToken && journal.committed>=migrationToken){
    telemetrySequence.restore(migrationFloor);sequenceKnown=true;migrationToken=0;
  }
  if(sequenceToken && journal.committed>=sequenceToken){
    telemetrySequence.commit(sequenceHighPending);sequenceToken=sequenceHighPending=0;
  }
  if(armToken && journal.committed<armToken && !journal.usable()){
    mission.armed=false;recoveryHold=true;
  }
  if(telemetryOn && sequenceKnown && !sequenceToken && !telemetrySequence.available() && telemetrySequence.next &&
     journal.usable() && !journal.criticalLost){
    FlightRecord reserve=flightSnapshot(LOG_PACKET_RANGE,UINT32_MAX);
    reserve.metadata.telemetryHigh=telemetrySequence.proposal();
    sequenceToken=persistCheckpoint(reserve);
    if(sequenceToken)sequenceHighPending=reserve.metadata.telemetryHigh;
  }
  if(state==ST_LANDED && mission.launched && !mission.complete && !completionAttempted && !cameraQueued && !camRecording &&
     (!camStartRequested || camStopTransmitted)) {
    completionAttempted=true;
    mission.complete=true;mission.armed=false;
    if(!logEvent(LOG_COMPLETE)) {mission.complete=false;recoveryHold=true;}
  }
  if(!calibrationPending && uint32_t(now-lastLogMs)>=200) {
    lastLogMs=now;logEvent(LOG_SAMPLE);
  }
  // Sparse recovery state, independent of the removable flight log.
  if(mission.launched && !mission.complete && journal.usable() && !calibrationPending &&
     uint32_t(now-lastRecoveryCheckpointMs)>=5000){
    lastRecoveryCheckpointMs=now;persistCheckpoint(flightSnapshot(LOG_CLOCK,UINT32_MAX));
  }
}

// ---------------- Init functions ----------------
bool setupLoRa() {
  Serial.print("# [SX1262] init... ");
  int16_t st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC_WORD, LORA_TX_POWER, 8);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("FAILED, code ");
    Serial.println(st);
    return false;
  }
  if (LORA_TCXO_VOLTAGE > 0) {
    st = radio.setTCXO(LORA_TCXO_VOLTAGE);
    if (st != RADIOLIB_ERR_NONE) {
      Serial.print("(setTCXO returned ");
      Serial.print(st);
      Serial.print(") ");
    }
  }
  radio.setRfSwitchPins(PIN_LORA_RXEN, PIN_LORA_TXEN);
  Serial.println("OK");
  return true;
}

bool setupBMP() {
  const uint8_t addrs[2] = { BMP_ADDR_A, BMP_ADDR_B };
  for (int attempt = 0; attempt < 3; attempt++) {
    for (int i = 0; i < 2; i++) {
      if (bmp.beginI2C(addrs[i], Wire2) == BMP5_OK) {
        bmp5_osr_odr_press_config osr={};
        if(bmp.getOSRMultipliers(&osr)!=BMP5_OK)continue;
        osr.osr_t = BMP5_OVERSAMPLING_2X;
        osr.osr_p = BMP5_OVERSAMPLING_8X;
        if(bmp.setOSRMultipliers(&osr)!=BMP5_OK ||
           bmp.setODRFrequency(BMP5_ODR_50_HZ)!=BMP5_OK)continue;
        // Poll a new-sample status bit; no new interrupt GPIO or wiring required.
        BMP581_InterruptConfig irq = {};
        irq.enable = BMP5_INTR_ENABLE; irq.drive = BMP5_INTR_PUSH_PULL;
        irq.polarity = BMP5_ACTIVE_HIGH; irq.mode = BMP5_LATCHED;
        irq.sources.drdy_en = BMP5_ENABLE;
        if (bmp.setInterruptConfig(&irq) != BMP5_OK)continue;
        // Bosch set_osr_odr_press_config() forces STANDBY for ODR >5 Hz.
        // Restore NORMAL after all configuration, otherwise DRDY never advances.
        if(bmp.setMode(BMP5_POWERMODE_NORMAL)!=BMP5_OK)continue;
        bmp5_powermode mode;
        if(bmp.getMode(&mode)!=BMP5_OK || mode!=BMP5_POWERMODE_NORMAL)continue;
        Serial.print("# BMP581 OK at 0x");
        Serial.println(addrs[i], HEX);
        return true;
      }
    }
    delay(300);
  }
  Serial.println("# BMP581 NOT FOUND (Wire2: SDA 25 / SCL 24)");
  return false;
}

bool setupBNO() {
  const uint8_t addrs[2] = { BNO_ADDR_A, BNO_ADDR_B };
  for (int attempt = 0; attempt < 3; attempt++) {
    for (int i = 0; i < 2; i++) {
      if (bno.begin_I2C(addrs[i], &Wire2)) {
        enableIMUReports();
        Serial.print("# BNO085 OK at 0x");
        Serial.println(addrs[i], HEX);
        return true;
      }
    }
    delay(300);
  }
  Serial.println("# BNO085 NOT FOUND (Wire2: SDA 25 / SCL 24)");
  return false;
}

bool setupRTC() {
#if USE_RTC
  if (!rtc.begin(&Wire)) {
    Serial.println("# DS3231 RTC NOT FOUND (Wire: SDA 18 / SCL 19). Using GNSS time only.");
    return false;
  }
  rtcValid=readDs3231(rtcLastUnix);
  rtcAnchorMs=rtcChangeMs=millis();
  Serial.println(rtcValid?"# RTC calendar valid; UTC trust requires GNSS/TIME or journal":"# RTC invalid; waiting for GNSS/TIME, no compile-time adjustment");
  return true;
#else
  return false;
#endif
}

void setupGPS() {
  Serial7.addMemoryForRead(gpsRxBuf, sizeof(gpsRxBuf));
  const uint32_t bauds[2] = { 38400, 9600 };
  for (int i = 0; i < 2; i++) {
    Serial7.begin(bauds[i]);
    uint32_t t0 = millis();
    bool dollar = false;
    while (millis() - t0 < 1500 && !dollar) {
      while (Serial7.available()) {
        if (Serial7.read() == '$') dollar = true;
      }
    }
    if (dollar) {
      Serial.print("# GNSS NMEA detected at ");
      Serial.print(bauds[i]);
      Serial.println(" baud");
      return;
    }
    Serial7.end();
  }
  Serial7.begin(38400);
  Serial.println("# GNSS: no NMEA seen (check TX/RX wiring on pins 28/29). Using 38400.");
}

// ---------------- Radio helpers ----------------
// RadioLib 6.x exposes getIrqStatus(), 7.x renamed it getIrqFlags()
static inline uint32_t radioIrq() {
#if defined(RADIOLIB_VERSION_MAJOR) && (RADIOLIB_VERSION_MAJOR >= 7)
  return radio.getIrqFlags();
#else
  return radio.getIrqStatus();
#endif
}

void radioListen() {
  if(!radioReady){radioListening=false;return;}
  rxFrameActive=false;
  radioListening = (radio.startReceive() == RADIOLIB_ERR_NONE);
}

bool startTx(const char* buf, int n, uint32_t now) {
  if(!radioReady)return false;
  radioListening = false;
  int16_t rc = radio.startTransmit((uint8_t*)buf, n);
  if (rc != RADIOLIB_ERR_NONE) {
    Serial.print("# TX start failed, code ");
    Serial.println(rc);
    radioListen();
    return false;
  }
  coexistence.sent();
  txBusy = true;
  txStartMs = now;
  txWaitMs = (uint32_t)(radio.getTimeOnAir(n) / 1000) + 30;   // airtime + margin
  return true;
}

uint8_t xorChecksum(const char* s) {
  uint8_t cs = 0;
  while (*s) cs ^= (uint8_t)*s++;
  return cs;
}

void queueAck(uint32_t seq, const char* status, const char* info, bool toUsb) {
  char body[160];
  snprintf(body, sizeof(body), "ACK,%u:%c,%lu,%s,%s", (unsigned)TEAM_ID, VEHICLE_ID, (unsigned long)seq, status, info);
  if(toUsb){
    char reply[180];
    snprintf(reply,sizeof(reply),"$%s*%02X",body,(unsigned)xorChecksum(body));
    Serial.println(reply); // machine-readable USB ACK, independent of LoRa/TX state
    return;
  }
  radioAckSequence=seq;
  ackLen = snprintf(ackBuf, sizeof(ackBuf), "$%s*%02X", body, (unsigned)xorChecksum(body));
  ackPending = true;
  ackDueMs = millis() + CMD_ACK_DELAY_MS;
}

// ---------------- Calibration ----------------
void calFinish(const char* status, const char* info) {
  snprintf(calStatus, sizeof(calStatus), "%s", status);
  snprintf(calInfo, sizeof(calInfo), "%s", info);
  calMask = 0;
  calReport = true;      // ACK goes out from handleLoRa()
  if(haveCmdSeq && lastCmdSeq==calSeq){
    snprintf(lastStatus,sizeof(lastStatus),"%s",status);
    snprintf(lastInfo,sizeof(lastInfo),"%s",info);
  }
}

bool calibrationStationary(uint32_t now) {
  // Pressure stability over the interval additionally rejects motion.
  // Permit measuring a bounded bias rather than requiring it already zero.
  return imuFresh && gyroValid && uint32_t(now-gyroMs)<=100 &&
    accelG>0.85f && accelG<1.15f && fabsf(verticalAccel)<2 &&
    fabsf(gx)<5 && fabsf(gy)<5 && fabsf(gz)<5;
}

void applyCalibration(const FlightCalibration& value,uint32_t now) {
  bool resetAltitude=!baroReferenceReady || value.pressure!=groundPressure || value.verticalBias!=vertical.bias;
  calibration=value;groundPressure=value.pressure;baroReferenceReady=true;
  gxOff=value.gyroX;gyOff=value.gyroY;gzOff=value.gyroZ;
  gx=gxRaw-gxOff;gy=gyRaw-gyOff;gz=gzRaw-gzOff;
  if(resetAltitude)vertical=VerticalEstimate();
  if(resetAltitude && baroSeen && uint32_t(now-baroReadMs)<=100){
    vertical.reset(now,value.verticalBias);
    vertical.baroMs=baroReadMs;
    altitude=altPrev=vertical.height=vertical.baroHeight=pressureToAltitude(pressurePa,groundPressure);
    velocity=0;
  }
  // This function is used on the pad or before restoring an interrupted flight.
  if(resetAltitude && !mission.launched){maxAlt=maxVel=0;launchCount=fallbackLaunchCount=0;}
}

bool beginCalibrationWrite(FlightCalibration value,bool fromCommand,uint32_t now) {
  if(calibrationPending || !value.valid())return false;
  value.generation=calibration.generation+1;
  if(!journal.usable()) {
    applyCalibration(value,now);
    if(fromCommand)calFinish("DONE","APPLIED_VOLATILE_STORAGE_UNAVAILABLE");
    Serial.println("# Calibration applied VOLATILE; flash unavailable");
    return true;
  }
  FlightRecord r=flightSnapshot(LOG_CALIBRATION,fromCommand?calSeq:UINT32_MAX);
  // Dedicated calibration payload, separate from transient sample measurements.
  r.groundPressure=value.pressure;r.ax=value.gyroX;r.ay=value.gyroY;r.az=value.gyroZ;
  r.voltage=value.verticalBias;r.metadata.calibrationGeneration=value.generation;
  calibrationToken=persistCheckpoint(r);
  if(!calibrationToken)return false;
  pendingCalibration=value;calibrationPending=true;calibrationFromCommand=fromCommand;
  calibrationQueuedMs=now;calMask=0;
  return true;
}

void serviceCalibrationStorage(uint32_t now) {
  if(!calibrationPending)return;
  if(journal.committed>=calibrationToken){
    applyCalibration(pendingCalibration,now);calibrationPending=false;
    if(calibrationFromCommand)calFinish("DONE","APPLIED_AND_PERSISTED");
  } else if(!journal.usable() || uint32_t(now-calibrationQueuedMs)>5000){
    calibrationPending=false;
    if(calibrationFromCommand)calFinish("FAIL","CAL_STORAGE_FAILED_OLD_PARAMETERS_RETAINED");
  }
}

void runCalibration(uint32_t now, bool gotBaro) {
  if (!calMask) return;
  if(!imuFresh || !gyroValid || uint32_t(now-gyroMs)>100 ||
     ((calMask&CAL_BARO) && (!baroSeen || uint32_t(now-baroReadMs)>100))){
    calFinish("FAIL","SENSOR_STALE_OR_UNRELIABLE");return;
  }
  if((state!=ST_PAD) || mission.launched || !calibrationStationary(now)){
    calFinish("FAIL","MOVED_OR_MISSION_ACTIVE");return;
  }
  if((calMask&CAL_BARO) && gotBaro){
    if(!calPN)calPressureMin=calPressureMax=pressurePa;
    calPressureMin=fminf(calPressureMin,pressurePa);calPressureMax=fmaxf(calPressureMax,pressurePa);
    calP+=pressurePa;++calPN;
    if(fabsf(pressureToAltitude(calPressureMin,calPressureMax))>0.75f){calFinish("FAIL","PRESSURE_UNSTABLE");return;}
  }
  if((calMask&CAL_GYRO) && gyroStamp!=calLastGyroStamp){
    calLastGyroStamp=gyroStamp;calGx+=gxRaw;calGy+=gyRaw;calGz+=gzRaw;++calGN;
  }
  if((calMask&CAL_ACCEL) && accelStamp!=calLastAccelStamp){
    calLastAccelStamp=accelStamp;calVertical+=verticalAccel;++calAN;
  }

  if (now - calStartMs < CAL_DURATION_MS) return;

  // Validate ALL requested sample sets before changing any active parameter.
  if((calMask&CAL_BARO) && calPN<100){calFinish("FAIL","INSUFFICIENT_FRESH_BARO");return;}
  if((calMask&CAL_GYRO) && calGN<100){calFinish("FAIL","INSUFFICIENT_FRESH_GYRO");return;}
  if((calMask&CAL_ACCEL) && calAN<100){calFinish("FAIL","INSUFFICIENT_FRESH_ACCEL");return;}
  FlightCalibration value=calibration;
  if(calMask&CAL_BARO)value.pressure=calP/calPN;
  if(calMask&CAL_GYRO){value.gyroX=calGx/calGN;value.gyroY=calGy/calGN;value.gyroZ=calGz/calGN;}
  if(calMask&CAL_ACCEL)value.verticalBias=calVertical/calAN;
  if(!beginCalibrationWrite(value,true,now))calFinish("FAIL","INVALID_CALIBRATION_OR_STORAGE_BUSY");
}

// ---------------- Command execution ----------------
const char* executeCommand(const char* name, const char* arg, char* info, size_t infoLen,
                           uint32_t seq, uint32_t now) {
  if(radioSwitch.pending && strcmp(name,"STATUS") && strcmp(name,"PING")){
    snprintf(info,infoLen,"RADIO_SWITCH_PENDING");return "REJECT";
  }
  if(rebootPending && strcmp(name,"STATUS") && strcmp(name,"PING") && strcmp(name,"REBOOT")){
    snprintf(info,infoLen,"REBOOT_PENDING");return "REJECT";
  }
  if((padResetToken || padResetResultPending) && strcmp(name,"STATUS") && strcmp(name,"PING")){
    snprintf(info,infoLen,"PAD_RESET_PENDING");return "REJECT";
  }
  if(counterResetToken && strcmp(name,"STATUS") && strcmp(name,"PING") &&
     (strcmp(name,"TELEM") || strcmp(arg,"0"))){
    snprintf(info,infoLen,"COUNTER_RESET_PENDING");return "REJECT";
  }
  if (!strcmp(name,"HELP")){
    snprintf(info,infoLen,"PING;STATUS;TELEM;RATE;CAL;TIME;ARM;RESETPAD;REBOOT;COUNTER;USB_ONLY_RESETCOUNT;PWM4;PWM5");return "OK";
  }
  if (!strcmp(name, "PING")) {
    if(!radioReady){snprintf(info,infoLen,"USB_OK;RADIO_UNAVAILABLE");return "OK";}
    snprintf(info, infoLen, "RSSI=%.0f;SNR=%.1f", radio.getRSSI(), radio.getSNR());
    return "OK";
  }

  if (!strcmp(name, "STATUS")) {
    if(!strcmp(arg,"RADIO")){
      snprintf(info,infoLen,"RADIO_CHANNEL=%s;MHZ=%.2f;SF=%u;SYNC=0x%02X;PENDING=%u",radioChannelName(radioSwitch.current),radioChannelMHz(radioSwitch.current),LORA_SF,LORA_SYNC_WORD,radioSwitch.pending);return "OK";
    }
    if(!strcmp(arg,"SERVO")){servos.status(info,infoLen);return "OK";}
    if(!strcmp(arg,"LINK")){
      snprintf(info,infoLen,"LINK=USB_AND_LORA;RADIO=%u;RX=%u;TX_BUSY=%u;BUILD=USB_CMD_2;SERVO=ACK_FIRST_1",radioReady,radioListening,txBusy);
      return "OK";
    }
    if(!strcmp(arg,"COUNTER")){
      snprintf(info,infoLen,"PACKET=%lu;NEXT=%lu;HIGH=%lu;EPOCH=%lu;TX=%u;RESET=%u",
        (unsigned long)packetCount,(unsigned long)telemetrySequence.next,
        (unsigned long)telemetrySequence.high,(unsigned long)journal.counterEpoch,telemetryOn,bool(counterResetToken));
      return "OK";
    }
    if(!strcmp(arg,"SD")){
      snprintf(info,infoLen,"SD=%u;FILE=%s;WR=%lu;DROP=%lu;SLOW=%lu",unsigned(sdLog.status),sdLog.filename,
        (unsigned long)sdLog.written,(unsigned long)sdLog.dropped,(unsigned long)sdLog.slowOperations);
      return "OK";
    }
    if(!strcmp(arg,"IMU")){
      snprintf(info,infoLen,"BNO=%u;CFG=%X;A=%u;G=%u;V=%u;Q=%u/%u/%u;AGE=%lu/%lu/%lu;DROP=%lu;DEC=%lu;RST=%lu;ERR=%lu",
        bnoOK,imuReportMask,imuAccel.fresh(now),imuGyro.fresh(now),imuGravity.fresh(now),
        imuAccel.accuracy,imuGyro.accuracy,imuGravity.accuracy,
        (unsigned long)imuAccel.ageForStatus(now),(unsigned long)imuGyro.ageForStatus(now),(unsigned long)imuGravity.ageForStatus(now),
        (unsigned long)min(bno.droppedReports(),uint32_t(999999)),(unsigned long)min(bno.decodeErrors,uint32_t(999999)),
        (unsigned long)min(imuResetCount,uint32_t(999999)),(unsigned long)min(imuConfigFailures,uint32_t(999999)));
      return "OK";
    }
    if (!strcmp(arg, "CAMERA")) {
      snprintf(info, infoLen, "REF=1;SR=%u;ST=%u;SF=%u;TT=%u;TF=%u;L=%lu;AS=%u;CA=NA;E=%u;R=%u",
               camStartRequested, camStartTransmitted, camStartFwAck,
               camStopTransmitted, camStopFwAck, (unsigned long)cameraLanding.landedMs,
               (unsigned)autoStopStatus, vertical.mode, vertical.ready && flightDataFresh);
      return "OK";
    }
    snprintf(info, infoLen, "ST=%u;TX=%u;RATE=%lu;CAM=%u;BMP=%u;BNO=%u;RTC=%u;SAT=%u;P=%.0f;SEQ=%u;HI=%lu;CAL=%lu;AV=%u",
             (unsigned)wireState(), (unsigned)telemetryOn, (unsigned long)telemetryPeriodMs,
             (unsigned)camRecording, (unsigned)bmpOK, (unsigned)bnoOK, (unsigned)rtcOK,
             (unsigned)gnssSats, groundPressure,packetSequenceStatus(),(unsigned long)telemetrySequence.high,
             (unsigned long)calibration.generation,baroReferenceReady && vertical.fresh(now));
    return "OK";
  }

  if (!strcmp(name,"PWM4") || !strcmp(name,"PWM5")) {
    // Existing global pending-operation gates above remain authoritative.
    // No new test mode or flight-state policy is introduced here.
    uint8_t channel=!strcmp(name,"PWM4")?0:1;
    ServoRequest request=servoRequest(servos.config[channel],arg);
    // Direct USB keeps its existing immediate reply. Invalid wireless requests
    // use the same validation/error response and cannot start a servo output.
    if(commandFromUsb || request.result!=ServoRequest::ACCEPT)
      return servos.test(channel,arg,info,infoLen);
    servoRadio.queue(channel,uint16_t(request.us),seq);
    snprintf(info,infoLen,"PWM%u_ACCEPTED_US=%u;FEEDBACK=NONE",channel+4,unsigned(request.us));
    return "OK";
  }

  if (!strcmp(name, "TELEM")) {
    if (!strcmp(arg,"1")) { telemetryOn = true; snprintf(info, infoLen, "TELEM_ON"); return "OK"; }
    if (!strcmp(arg,"0")) {
      // Do not accept OFF before recovery can establish the flight safety state.
      if (!recoveryDone) { snprintf(info, infoLen, "RECOVERY_SCANNING"); return "REJECT"; }
      if (inFlight()) { snprintf(info, infoLen, "TELEM_REQUIRED_IN_FLIGHT"); return "REJECT"; }//WTF do we need this??(sajag)
      telemetryOn = false;
      snprintf(info, infoLen, "TELEM_OFF");
      return "OK";
    }
    snprintf(info, infoLen, "ARG");
    return "ERR";
  }

  if (!strcmp(name, "RATE")) {
    if(!decimalU32(arg)){snprintf(info,infoLen,"ARG_UNSIGNED_MS");return "ERR";}
    long v = atol(arg);
    if (v < RATE_MIN_MS || v > RATE_MAX_MS) { snprintf(info, infoLen, "RANGE_%d_%d", RATE_MIN_MS, RATE_MAX_MS); return "ERR"; }
    telemetryPeriodMs = (uint32_t)v;
    snprintf(info, infoLen, "RATE=%ld", v);
    return "OK";
  }

  if (!strcmp(name, "CAM")) {
    if (!USE_RUNCAM) { snprintf(info, infoLen, "CAM_DISABLED"); return "ERR"; }
    if (strcmp(arg, "1") && strcmp(arg, "0")) { snprintf(info, infoLen, "ARG"); return "ERR"; }
    bool start = !strcmp(arg, "1");
    if (start && state == ST_LANDED) { snprintf(info, infoLen, "NEW_MISSION_REQUIRED"); return "REJECT"; }//interesting sajag
    if(cameraQueued || cameraResultPending){snprintf(info,infoLen,"CAM_BUSY");return "REJECT";}
    if(!recoveryDone || (!journal.usable() && mission.launched)){
      snprintf(info,infoLen,"CAM_RECOVERY_OR_STORAGE_HOLD");return "REJECT";
    }
    cameraReplyUsb=commandFromUsb;
    bool priorRequested=camStartRequested,priorStart=camStartTransmitted,priorStartAck=camStartFwAck;
    bool priorStop=camStopTransmitted,priorStopAck=camStopFwAck,priorAttempted=cameraLanding.attempted;
    AutoStopStatus priorAuto=autoStopStatus;
    if (start) {
      camStartRequested = true; camStartFwAck = false;
      camStartTransmitted = false;
      camStopTransmitted = camStopFwAck = false; cameraLanding.attempted = false;
    }
    if (!start) { cameraLanding.attempted = true; autoStopStatus = AUTO_MANUAL; camStopFwAck = false; }
    if(journal.usable()){
      if(!queueCamera(start,false,seq,now)){
        camStartRequested=priorRequested;camStartTransmitted=priorStart;camStartFwAck=priorStartAck;
        camStopTransmitted=priorStop;camStopFwAck=priorStopAck;cameraLanding.attempted=priorAttempted;
        autoStopStatus=priorAuto;
        snprintf(info,infoLen,"CAM_STORAGE_FAILED");return "FAIL";
      }
      snprintf(info,infoLen,"CAM_PERSISTING");return "OK";
    }
    // Preserve manual bench control when flash is not configured. This mode is
    // explicitly unlogged; arming/mission recovery is unavailable, not simulated.
    if (!(start ? camStartRecording() : camStopRecording())) {
      if (!start) autoStopStatus = AUTO_FAILED;
      snprintf(info, infoLen, "CAM_UART_FAILED_OR_DISABLED"); return "FAIL";
    }
    if (start) camStartTransmitted = true; else camStopTransmitted = true;
    camRecording = start;
    logEvent(LOG_CAM_RESULT,seq);
    snprintf(info, infoLen, "%s", start ? "CAM_REC_START_SENT" : "CAM_REC_STOP_SENT");
    return "OK";
  }

  if (!strcmp(name, "CAMPWR")) {
    if (*arg) { snprintf(info, infoLen, "ARG"); return "ERR"; }
    if (camRecording || cameraQueued || !recoveryDone || mission.launched) { snprintf(info, infoLen, "CAM_RECORDING_REQUEST_ACTIVE"); return "REJECT"; }
    logEvent(LOG_CAM_INTENT,seq);
    if (!camPowerButton()) { snprintf(info, infoLen, "CAM_UART_FAILED_OR_DISABLED"); return "FAIL"; }
    logEvent(LOG_CAM_RESULT,seq);
    snprintf(info, infoLen, "CAM_POWER_BUTTON_SENT");
    return "OK";
  }

  if(!strcmp(name,"RADIO")){
    RadioChannel channel=radioChannel(arg);
    if(channel==RadioChannel::INVALID){snprintf(info,infoLen,"ARG_MAIN_OR_BACKUP");return "ERR";}
    if(!radioReady){snprintf(info,infoLen,"RADIO_UNAVAILABLE");return "ERR";}
    if(txBusy || ackPending){snprintf(info,infoLen,"RADIO_BUSY");return "REJECT";}
    if(channel==radioSwitch.current){snprintf(info,infoLen,"RADIO_CHANNEL=%s;MHZ=%.2f",radioChannelName(channel),radioChannelMHz(channel));return "OK";}
    radioSwitch.queue(channel,seq,commandFromUsb,now);
    snprintf(info,infoLen,"RADIO_SWITCH_PENDING=%s",radioChannelName(channel));return "OK";
  }
  if (!strcmp(name, "CAL")) {
    uint8_t mask = 0;
    if (!strcmp(arg, "BARO")) mask = CAL_BARO;
    else if (!strcmp(arg, "GYRO")) mask = CAL_GYRO;
    else if (!strcmp(arg, "ACCEL")) mask = CAL_ACCEL;
    else if (!strcmp(arg, "ALL")) mask = CAL_BARO | CAL_GYRO | CAL_ACCEL;
    else { snprintf(info, infoLen, "ARG"); return "ERR"; }

    if(!recoveryDone){snprintf(info,infoLen,"RECOVERY_SCANNING");return "REJECT";}
    if((state!=ST_PAD) || mission.launched){snprintf(info,infoLen,"MISSION_ACTIVE_OR_NOT_ON_PAD");return "REJECT";}
    if (calMask || calReport || calibrationPending) { snprintf(info, infoLen, "CAL_BUSY"); return "REJECT"; }
    if ((mask & CAL_BARO) && !bmpOK) { snprintf(info, infoLen, "NO_BMP"); return "ERR"; }
    if (!bnoOK || !imuFresh || !gyroValid || uint32_t(now-gyroMs)>100) { snprintf(info, infoLen, "IMU_STALE_OR_UNRELIABLE"); return "REJECT"; }
    if(!calibrationStationary(now)){snprintf(info,infoLen,"KEEP_ROCKET_STILL");return "REJECT";}
    if(!(mask&CAL_BARO) && !baroReferenceReady){snprintf(info,infoLen,"CAL_BARO_FIRST");return "REJECT";}

    calP = calGx = calGy = calGz = 0;
    calPN = calGN = 0;
    calAN=0;calVertical=0;calLastGyroStamp=gyroStamp;calLastAccelStamp=accelStamp;
    calSeq = seq;calReplyUsb=commandFromUsb;
    calStartMs = now;
    calMask = mask;
    snprintf(info, infoLen, "CAL_STARTED_%lus", (unsigned long)(CAL_DURATION_MS / 1000));
    return "OK";
  }

  if (!strcmp(name, "TIME")) {
#if USE_RTC
    if (!rtcOK) { snprintf(info, infoLen, "NO_RTC"); return "ERR"; }
    if (!recoveryDone || (mission.launched && !mission.complete)) { snprintf(info, infoLen, "MISSION_CLOCK_LOCKED"); return "REJECT"; }
    uint32_t u = strtoul(arg, NULL, 10);
    if (!decimalU32(arg) || u < 1700000000UL || u>=4102444800UL) { snprintf(info, infoLen, "ARG"); return "ERR"; }
    rtc.adjust(DateTime(u));
    rtcSynced = true;
    uint32_t checked=0;
    rtcValid=readDs3231(checked) && checked>=u && checked-u<=1;
    rtcTrusted=rtcValid;rtcLastUnix=checked;rtcChangeMs=now;
    if(!rtcValid){snprintf(info,infoLen,"RTC_VERIFY_FAILED");return "FAIL";}
    logEvent(LOG_CLOCK);
    snprintf(info, infoLen, "RTC_SET");
    return "OK";
#else
    snprintf(info, infoLen, "NO_RTC");
    return "ERR";
#endif
  }

  if (!strcmp(name, "REBOOT")) {
    if(!recoveryDone){snprintf(info,infoLen,"RECOVERY_SCANNING");return "REJECT";}
    if(state!=ST_PAD || mission.armed || mission.launched){snprintf(info,infoLen,"REBOOT_REQUIRES_UNARMED_PAD");return "REJECT";}
    if(cameraQueued || cameraResultPending){snprintf(info,infoLen,"CAM_BUSY");return "REJECT";}
    if(calMask || calibrationPending){snprintf(info,infoLen,"CAL_BUSY");return "REJECT";}
    if(!rebootPending){rebootPending=true;rebootRequestedMs=now;rebootReady=false;}
    snprintf(info, infoLen, "REBOOTING");
    return "OK";
  }

  if(!strcmp(name,"RESETPAD")){
    if(strcmp(arg,"CONFIRM")){snprintf(info,infoLen,"USE_RESETPAD_CONFIRM");return "ERR";}
    if(!recoveryDone || !journal.idle() || journal.criticalLost){snprintf(info,infoLen,"STORAGE_BUSY_OR_NOT_RECOVERABLE");return "REJECT";}
    if(state!=ST_PAD && state!=ST_LANDED){snprintf(info,infoLen,"IN_FLIGHT_RESET_BLOCKED");return "REJECT";}
    if(state==ST_PAD && mission.launched && !mission.complete){snprintf(info,infoLen,"ACTIVE_MISSION_RECOVERY_PRESERVED");return "REJECT";}
    if(calMask || calReport || calibrationPending || sequenceToken || migrationToken){snprintf(info,infoLen,"CAL_OR_COUNTER_BUSY");return "REJECT";}
    if(camRecording || cameraQueued || cameraResultPending){snprintf(info,infoLen,"STOP_RECORDING_BEFORE_PAD_RESET");return "REJECT";}
    if(missionId==UINT32_MAX){snprintf(info,infoLen,"MISSION_ID_EXHAUSTED");return "ERR";}
    FlightRecord r=padMissionResetRecord(flightSnapshot(LOG_MISSION_RESET,seq),missionId+1);
    padResetToken=persistCheckpoint(r);
    if(!padResetToken){snprintf(info,infoLen,"PAD_RESET_NOT_QUEUED");return "FAIL";}
    mission.armed=false; // immediately inhibit launch while the new PAD record commits
    padResetSequence=seq;padResetQueuedMs=now;padResetFromUsb=commandFromUsb;
    snprintf(info,infoLen,"PAD_RESET_PERSISTING");return "OK";
  }
  if (!strcmp(name, "ARM")) {
    if (!recoveryDone || !journal.usable() || journal.criticalLost) {
      snprintf(info,infoLen,"STORAGE_NOT_RECOVERABLE");return "REJECT";
    }
    if(!sequenceKnown){snprintf(info,infoLen,"COUNTER_LEGACY_BASELINE_REQUIRED");return "REJECT";}
    if(!telemetrySequence.next){snprintf(info,infoLen,"COUNTER_EXHAUSTED");return "REJECT";}
    if (recoveryHold && strcmp(arg,"CONFIRM")) {
      snprintf(info,infoLen,"REVIEW_RECOVERY_THEN_ARM_CONFIRM");return "REJECT";
    }
    if(*arg && strcmp(arg,"CONFIRM")){snprintf(info,infoLen,"ARG_CONFIRM_OR_NONE");return "ERR";}
    if(mission.launched && !mission.complete){snprintf(info,infoLen,"ACTIVE_MISSION_RECOVERY_PRESERVED");return "REJECT";}
    if(state!=ST_PAD && state!=ST_LANDED){snprintf(info,infoLen,"MISSION_STATE_UNSAFE");return "REJECT";}
    if(!imuFresh || !gyroValid || uint32_t(now-gyroMs)>100){snprintf(info,infoLen,"IMU_STALE_OR_UNRELIABLE");return "REJECT";}
    if(!stationary){snprintf(info,infoLen,"KEEP_ROCKET_STILL");return "REJECT";}
    if(!baroReferenceReady || !flightDataFresh){snprintf(info,infoLen,"BARO_REFERENCE_NOT_READY");return "REJECT";}
    if(camRecording || cameraQueued || cameraResultPending){snprintf(info,infoLen,"STOP_RECORDING_BEFORE_ARM");return "REJECT";}
    if(calMask || calReport || calibrationPending){snprintf(info,infoLen,"CAL_BUSY");return "REJECT";}
    mission=MissionClock();mission.armed=true;++missionId;completionAttempted=false;
    servos.newMission(); // manual tests never clear the automatic intent latch
    landingRtc=0;landingElapsed=UNKNOWN_ELAPSED;recoveredMission=recoveryHold=cameraRecoveryInhibit=false;
    setState(ST_PAD);
    baroReferenceReady=false; // intentional new mission, never a telemetry side effect
    // PAD stays unarmed for detection until the new mission record is committed.
    armToken=logEvent(LOG_ARM,seq);
    if(!armToken){mission.armed=false;snprintf(info,infoLen,"ARM_NOT_PERSISTED");return "FAIL";}
    snprintf(info, infoLen, "MISSION_INITIALIZING"); return "OK";
  }

  if(!strcmp(name,"COUNTER")) {
    // One-time migration only: old version-1 logs never stored packet numbers.
    // Operator must supply a known upper bound across ALL old transmissions.
    if(sequenceKnown || migrationToken || !recoveryDone || !journal.usable()){
      snprintf(info,infoLen,"COUNTER_MIGRATION_NOT_AVAILABLE");return "REJECT";
    }
    if(!decimalU32(arg)){snprintf(info,infoLen,"ARG_LAST_USED_UPPER_BOUND");return "ERR";}
    uint32_t floor=strtoul(arg,NULL,10);
    if(!floor || floor<telemetrySequence.high || floor==UINT32_MAX){snprintf(info,infoLen,"COUNTER_RANGE");return "ERR";}
    FlightRecord r=flightSnapshot(LOG_PACKET_RANGE,seq);r.metadata.telemetryHigh=floor;
    migrationToken=persistCheckpoint(r);migrationFloor=floor;
    snprintf(info,infoLen,"%s",migrationToken?"COUNTER_BASELINE_PERSISTING":"COUNTER_STORAGE_FAILED");
    return migrationToken?"OK":"FAIL";
  }

  snprintf(info, infoLen, "UNKNOWN");
  return "ERR";
}

// Validates and runs one received packet
void handleCommand(char* s, uint32_t now, bool fromUsb) {
  if (!commandAddressMatches(s,strlen(s),TEAM_ID,VEHICLE_ID)) {
    if(fromUsb)Serial.println("# USB rejected: invalid command frame/checksum or wrong vehicle");
    return;
  }
  char* star = strrchr(s, '*');
  if (!star) return;

  uint8_t cs = 0;
  for (char* p = s + 1; p < star; p++) cs ^= (uint8_t)*p;
  if (cs != (uint8_t)strtoul(star + 1, NULL, 16)) { Serial.println("# CMD bad checksum"); return; }
  *star = 0;

  char* t[6] = { 0 };
  int nt = 0;
  char* save = NULL;
  for (char* tok = strtok_r(s + 1, ",", &save); tok && nt < 6; tok = strtok_r(NULL, ",", &save)) t[nt++] = tok;
  if (nt < 5) return;                                          // CMD,team,key,seq,name
  if (!decimalU32(t[3])) {if(fromUsb)Serial.println("# USB rejected: invalid sequence");return;}
  uint32_t seq = strtoul(t[3], NULL, 10);
  if (!decimalU32(t[2]) || strtoul(t[2], NULL, 10) != CMD_KEY) {
    if(fromUsb || !ackPending)queueAck(seq,"REJECT","AUTH_KEY",fromUsb);
    return;
  }
  const char* name = t[4];
  const char* arg  = (nt > 5) ? t[5] : "";

  if (ackPending && !fromUsb) {Serial.println("# CMD busy: previous radio ACK pending");return;}

  Serial.print("# CMD "); Serial.print(name); Serial.print(' '); Serial.println(arg);

  if (haveCmdSeq && seq == lastCmdSeq) {      // retransmission: repeat the ACK, do not run again
    queueAck(seq, lastStatus, lastInfo,fromUsb);
    return;
  }

  if(commandHistory.contains(seq)){
    queueAck(seq,"REJECT","DELAYED_DUPLICATE",fromUsb);return;
  }
  commandHistory.add(seq);
  char info[128] = "";
  commandFromUsb=fromUsb;
  const char* status = executeCommand(name, arg, info, sizeof(info), seq, now);
  commandFromUsb=false;
  lastCmdSeq = seq;
  haveCmdSeq = true;
  snprintf(lastStatus, sizeof(lastStatus), "%s", status);
  snprintf(lastInfo, sizeof(lastInfo), "%s", info);
  queueAck(seq, status, info,fromUsb);
  if (!strcmp(name, "CAM") && !strcmp(status, "OK") && !cameraQueued) {
    if (!strcmp(arg, "1")) camStartFwAck = true;
    else camStopFwAck = true;
  }
}

void pollRadioRx(uint32_t now) {
  if (!radioListening) return;
  if (now - lastRxPollMs < CMD_RX_POLL_MS) return;
  lastRxPollMs = now;
  if (!(radioIrq() & RADIOLIB_SX126X_IRQ_RX_DONE)) return;

  uint8_t buf[256];
  size_t len = radio.getPacketLength();
  if (len > 0 && len < sizeof(buf)) {
    if (radio.readData(buf, len) == RADIOLIB_ERR_NONE) {
      buf[len] = 0;
      if (validCommandFrame((const char*)buf, len)) {
        if(commandAddressMatches((const char*)buf,len,TEAM_ID,VEHICLE_ID=='R'?'C':'R'))
          coexistence.heardPeer(now); // protect the other vehicle's ACK window
        handleCommand((char*)buf, now,false);
      }
      else {
        char peer=fleetDownlinkVehicle((const char*)buf,len,TEAM_ID);
        if(peer && peer!=VEHICLE_ID)coexistence.heardPeer(now);
      }
    }
  }
  radioListen();     // re-arm receive
}

// ---------------- Telemetry ----------------
void sendTelemetry(uint32_t now) {
  // Final gate: no caller may spend a sequence or transmit routine data while OFF.
  if(!telemetryOn || counterResetToken)return;
  // Never transmit a numbered frame until recovery and range commitment finish.
  if(!recoveryDone || !sequenceKnown || !telemetrySequence.available())return;
  packetCount=telemetrySequence.take();

  const char* timeStr = gnssTimeFresh ? gnssTime : (rtcValid ? rtcHms : gnssTime);
  uint8_t st = wireState();

  int n = snprintf(txBuf, sizeof(txBuf),
    "%u:%c,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%s,%.6f,%.6f,%.2f,%u,"
    "%.2f;%.2f;%.2f,%.2f;%.2f;%.2f,%u,%s;E=%lu",
    (unsigned)TEAM_ID, VEHICLE_ID,
    (unsigned long)timestampMs(now),
    (unsigned long)packetCount,
    baroReferenceReady && baroSeen && uint32_t(now-baroReadMs)<=100 &&
      vertical.fresh(now) && vertical.mode && isfinite(altitude)?altitude:NAN,
    baroSeen && uint32_t(now-baroReadMs)<=100?pressurePa:NAN,
    baroSeen && uint32_t(now-baroReadMs)<=100?tempC:NAN,
    readBusVoltage(),
    timeStr,
    gnssLat,
    gnssLon,
    gnssAlt,
    (unsigned)gnssSats,
    telemAx, telemAy, telemAz,
    telemGxRaw-gxOff, telemGyRaw-gyOff, telemGzRaw-gzOff,
    (unsigned)st,
    "1",(unsigned long)journal.counterEpoch); // source ID: state is numeric field 14, no trailing PAD text

  if (n <= 0 || n >= (int)sizeof(txBuf)) {
    Serial.println("# Packet too long, skipped");
    return;
  }

  // Checksum covers the CSV body, including the explicit team:vehicle address.
  if(n+3 >= int(sizeof(txBuf)))return;
  snprintf(txBuf+n,sizeof(txBuf)-n,"*%02X",xorChecksum(txBuf));
  n+=3;
  if (!radioReady || startTx(txBuf, n, now)) {
    Serial.println(txBuf);
#if USE_EEPROM
    eepromLogPacket(txBuf, n);
#endif
  }
}

// The first radio ACK must finish transmission before manual PWM begins.
// A later DONE reports issued PWM only. A motor-induced reset can still prevent
// DONE; acceptance is never represented as physical movement or deployment.
void serviceWirelessServo(){
  if(servoRadio.ready(txBusy)){
    char pulse[12],info[96];
    snprintf(pulse,sizeof(pulse),"%u",servoRadio.pulseUs);
    const char* status=servos.test(servoRadio.channel,pulse,info,sizeof(info));
    servoRadio.complete();
    // Configuration is immutable, so a validated queued request normally succeeds.
    servoRadio.ackFailed=strcmp(status,"OK")!=0;
  }
  if(!servoRadio.resultPending || txBusy || ackPending)return;
  char info[96];
  if(servoRadio.ackFailed)snprintf(info,sizeof(info),"PWM%u_ACK_OR_OUTPUT_FAILED",servoRadio.channel+4);
  else snprintf(info,sizeof(info),"PWM%u_CMD_US=%u;FEEDBACK=NONE",servoRadio.channel+4,servoRadio.pulseUs);
  const char* result=servoRadio.ackFailed?"FAIL":"DONE";
  queueAck(servoRadio.sequence,result,info,false);
  if(haveCmdSeq && lastCmdSeq==servoRadio.sequence){
    snprintf(lastStatus,sizeof(lastStatus),"%s",result);
    snprintf(lastInfo,sizeof(lastInfo),"%s",info);
  }
  servoRadio.resultPending=false;
}

// Non-blocking. Order: finish TX -> listen for commands -> ACK -> telemetry
void serviceCommandReplies(){
  if (calReport && (calReplyUsb || !ackPending)) {           // calibration finished: send the second ACK
    queueAck(calSeq, calStatus, calInfo,calReplyUsb);
    calReport = false;
  }
  if(cameraResultPending && (cameraReplyUsb || !ackPending)){
    queueAck(cameraCommandSeq,cameraResultStatus,cameraResultInfo,cameraReplyUsb);
    if(haveCmdSeq && cameraCommandSeq==lastCmdSeq){
      snprintf(lastStatus,sizeof(lastStatus),"%s",cameraResultStatus);
      snprintf(lastInfo,sizeof(lastInfo),"%s",cameraResultInfo);
    }
    if(!strcmp(cameraResultStatus,"DONE")){
      if(cameraQueuedStart)camStartFwAck=true;else camStopFwAck=true;
    }
    cameraResultPending=false;
  }

  if(padResetResultPending && (padResetFromUsb || !ackPending)){
    queueAck(padResetSequence,padResetStatus,padResetInfo,padResetFromUsb);
    if(haveCmdSeq && lastCmdSeq==padResetSequence){
      snprintf(lastStatus,sizeof(lastStatus),"%s",padResetStatus);
      snprintf(lastInfo,sizeof(lastInfo),"%s",padResetInfo);
    }
    padResetResultPending=false;
  }
}

void serviceRadioSwitch(uint32_t now){
  if(!radioSwitch.ready(now,txBusy || ackPending))return;
  const uint32_t sequence=radioSwitch.sequence;
  const bool usb=radioSwitch.fromUsb;
  const RadioChannel previous=radioSwitch.current;
  bool ok=!radioSwitch.ackFailed && uint32_t(now-radioSwitch.queuedMs)<=5000;
  int16_t rc=-1;
  if(ok){
    rc=radio.standby();radioListening=false;rxFrameActive=false;
    if(rc==RADIOLIB_ERR_NONE)rc=radio.setFrequency(radioChannelMHz(radioSwitch.target));
    ok=rc==RADIOLIB_ERR_NONE;
    if(ok){radioSwitch.current=radioSwitch.target;rc=radio.startReceive();radioListening=rc==RADIOLIB_ERR_NONE;ok=radioListening;}
    if(!ok){radio.setFrequency(radioChannelMHz(previous));radioSwitch.current=previous;radioListen();}
  }
  radioSwitch.pending=false;
  char info[128];
  if(ok)snprintf(info,sizeof(info),"RADIO_CHANNEL=%s;MHZ=%.2f;SF=%u;SYNC=0x%02X",radioChannelName(radioSwitch.current),radioChannelMHz(radioSwitch.current),LORA_SF,LORA_SYNC_WORD);
  else snprintf(info,sizeof(info),"RADIO_SWITCH_FAILED;MHZ=%.2f;RC=%d;LINK=UNKNOWN",radioChannelMHz(radioSwitch.current),rc);
  const char* status=ok?"DONE":"FAIL";
  queueAck(sequence,status,info,usb);
  if(haveCmdSeq && lastCmdSeq==sequence){
    snprintf(lastStatus,sizeof(lastStatus),"%s",status);snprintf(lastInfo,sizeof(lastInfo),"%s",info);
  }
}

void sendSystemStatus(uint32_t now){
    char elapsed[24],body[190];
    if(mission.quality==TIME_UNCERTAIN)snprintf(elapsed,sizeof(elapsed),"NA");
    else snprintf(elapsed,sizeof(elapsed),"%llu",(unsigned long long)mission.elapsed);
    snprintf(body,sizeof(body),"SYS,%u:%c,%lu,%u,%s,%u,%u,%u,%u,%.1f,%.1f,%lu,%u,%u,%u,%u,%lu",
      TEAM_ID,VEHICLE_ID,(unsigned long)missionId,mission.quality,elapsed,
      !recoveryDone?0:recoveryHold?4:mission.complete?2:recoveredMission?3:1,
      journal.criticalLost && journal.usable()?FlightJournal<FlashHardware>::FAULT:journal.status,
      rtcValid,ina219.valid,telemetryOn && ina219.valid?ina219.current:NAN,
      telemetryOn && ina219.valid?ina219.power:NAN,
      (unsigned long)journal.dropped,mission.armed,(unsigned)autoStopStatus,telemetryOn,packetSequenceStatus(),(unsigned long)journal.counterEpoch);
    int n=snprintf(txBuf,sizeof(txBuf),"$%s*%02X",body,xorChecksum(body));
    if(n>0 && n<int(sizeof(txBuf))){Serial.println(txBuf);if(radioReady)startTx(txBuf,n,now);}
}

void handleLoRa(uint32_t now) {
  serviceCommandReplies();
  if(telemetryOn)telemetrySessionStarted=true; // also preserves in-flight recovery policy
  if(!radioReady){
    if(rebootPending)return;
    if(telemetrySessionStarted && uint32_t(now-lastSystemMs)>=5000){lastSystemMs=now;sendSystemStatus(now);return;}
    if(routineTelemetryDue(telemetryOn,now,lastTelemetryMs,telemetryPeriodMs)){
      lastTelemetryMs=now;sendTelemetry(now);
    }
    return;
  }
  if (txBusy) {
    if (now - txStartMs >= txWaitMs) {
      radio.finishTransmit();
      txBusy = false;
      lastTxFinishedMs=now;
      radioListen();            // back to listening after every transmission
    }
    return;
  }

  pollRadioRx(now);

  // Protect a valid incoming header until RX_DONE or a bounded RF timeout.
  if(radioListening && (radioIrq()&RADIOLIB_SX126X_IRQ_HEADER_VALID)) {
    if(!rxFrameActive){
      rxFrameActive=true;rxFrameSince=now;
      rxFrameWaitMs=uint32_t(radio.getTimeOnAir(255)/1000)+30;
    }
    if(uint32_t(now-rxFrameSince)<rxFrameWaitMs)return;
    radioListen();return;
  }
  rxFrameActive=false;
  if (ackPending) {
    if ((int32_t)(now - ackDueMs) >= 0) {
      bool started=startTx(ackBuf, ackLen, now);
      servoRadio.ackStarted(radioAckSequence,started);
      radioSwitch.acceptanceTx(radioAckSequence,started);
      if (started) { Serial.print("# "); Serial.println(ackBuf); }
      ackPending = false;
    }
    return;
  }

  if(rebootPending)return; // ACK was serviced above; no new routine TX during shutdown
  if(radioSwitch.pending)return; // preserve a quiet acceptance-to-retune window
  if(uint32_t(now-lastTxFinishedMs)<250)return;

  // Keep a command receive window before the extra diagnostic downlink. In a
  // busy link this occasionally defers a telemetry slot, never changes its layout.
  bool systemDue=telemetrySessionStarted && uint32_t(now-lastSystemMs)>=5000;
  bool sensorDue=routineTelemetryDue(telemetryOn,now,lastTelemetryMs,telemetryPeriodMs);
  if((systemDue || sensorDue) && !coexistence.ready(now))return;
  if(!systemDue && routineTelemetryDue(telemetryOn,now,lastTelemetryMs,telemetryPeriodMs)) {
    lastTelemetryMs = now;
    sendTelemetry(now);
    return;
  }
  if(systemDue && uint32_t(now-lastTxFinishedMs)>=250){
    lastSystemMs=now;
    sendSystemStatus(now);
  }
}

// ACK first, then give the SD queue and NOR checkpoints a bounded drain window.
void serviceReboot(uint32_t now){
  if(!rebootPending)return;
  sdLog.requestClose();
  if(!txBusy && !ackPending)sdLog.service(now);
  bool drained=sdLog.stopped() && (journal.idle() || !journal.usable());
  if((drained || uint32_t(now-rebootRequestedMs)>=2000) && !txBusy && !ackPending){
    if(!rebootReady){
      rebootReady=true;rebootReadyMs=now;
      Serial.println(drained?"# REBOOT: restarting after storage shutdown":"# REBOOT: storage drain timed out; latest unsynced SD rows may be lost");
      return;
    }
    if(uint32_t(now-rebootReadyMs)>=50){
      asm volatile("dsb" ::: "memory");
      SCB_AIRCR=0x05FA0004; // Cortex-M7 SYSRESETREQ, not the programming button
      asm volatile("dsb" ::: "memory");
    }
  }
}
// Bounded, USB-only bench commands. Radio authentication/parser are unchanged.
void serviceUsbConsole(uint32_t now) {
  if(counterResetToken){
    if(journal.committed>=counterResetToken){
      journal.counterEpoch=counterResetToken;
      journal.telemetryHigh=journal.damagedSlots=0;journal.haveLegacy=false;
      telemetrySequence=TelemetrySequence();packetCount=0;sequenceKnown=true;
      counterResetToken=0;
      Serial.println("# RESETCOUNT DONE: count=0; next=1; telemetry remains OFF");
    } else if(!journal.usable()){
      // Outcome may be ambiguous: do not transmit until reboot recovery resolves it.
      Serial.println("# RESETCOUNT FAILED: storage fault; reboot for recovery");
      sequenceKnown=false;counterResetToken=0;
    }
  }
  static UsbCommandLine input;
  if(!Serial.available() && input.expire(now))Serial.println("# USB partial command discarded: enable Newline or CRLF in Serial Monitor");
  for(uint8_t budget=0;budget<32 && Serial.available();++budget){
    auto result=input.feed(Serial.read(),now);
    if(result==UsbCommandLine::INVALID){Serial.println("# USB rejected: invalid/oversized line");continue;}
    if(result!=UsbCommandLine::LINE)continue;
    if(input.text[0]=='$'){handleCommand(input.text,now,true);continue;}
    ConsoleCommand parsed;
    if(!parseConsoleCommand(input.text,parsed)){Serial.println("# USB rejected: use NAME,arg or NAME arg; one argument only");continue;}
    if(!strcmp(parsed.name,"HELP")){
      Serial.println("# USB/GCS commands: PING STATUS STATUS,LINK STATUS,SD STATUS,IMU STATUS,COUNTER TELEM,1 TELEM,0 RATE,500 CAL,BARO CAL,GYRO CAL,ACCEL CAL,ALL TIME,<unix> ARM RESETPAD,CONFIRM REBOOT");
      Serial.println("# USB only: RESETCOUNT CONFIRM. Rocket also supports CAM,1 CAM,0 STATUS,CAMERA CAMPWR PWM4,<us|SAFE|TEST90> PWM5,<us|SAFE|TEST90> STATUS,SERVO. All flight safety checks apply.");
      Serial.println("# PWM4 continuous servo: PWM4 SPIN / PWM4 REVERSE / PWM4 STOP. PWM5 remains independently controlled.");
      continue;
    }
    if(!strcmp(parsed.name,"RESETCOUNT")){
      if(strcmp(parsed.arg,"CONFIRM")){Serial.println("# USB rejected: RESETCOUNT CONFIRM required");continue;}
          if(counterResetToken)Serial.println("# RESETCOUNT PENDING");
          else if(!recoveryDone || !journal.idle() || journal.criticalLost ||
                  !sequenceKnown || sequenceToken || migrationToken || calibrationPending || calMask || calReport || rebootPending || padResetToken || padResetResultPending || cameraQueued || cameraResultPending ||
                  (state!=ST_PAD && !(state==ST_LANDED && mission.complete)) || mission.armed ||
                  (mission.launched && !mission.complete) || telemetryOn || txBusy || ackPending ||
                  !journal.nextSequence || journal.nextSequence==UINT32_MAX){
            Serial.println("# RESETCOUNT REJECT: need unarmed PAD or completed LANDED, telemetry OFF, idle healthy recovery store, no active mission/calibration");
          } else {
            FlightRecord r=flightSnapshot(LOG_COUNTER_RESET,UINT32_MAX);
            r.metadata.telemetryHigh=0;
            counterResetToken=persistCheckpoint(r);
            Serial.println(counterResetToken?"# RESETCOUNT PENDING: waiting for durable commit":"# RESETCOUNT FAILED: not queued");
          }

      continue;
    }
    // Local operator input is converted into our existing authenticated envelope.
    // Raw framed input above is never rewritten and still validates its own key.
    do{++usbCommandSequence;}while((haveCmdSeq && usbCommandSequence==lastCmdSeq) || commandHistory.contains(usbCommandSequence));
    char body[100],frame[100];
    int n=snprintf(body,sizeof(body),"CMD,%u:%c,%u,%lu,%s%s%s",TEAM_ID,VEHICLE_ID,CMD_KEY,
      (unsigned long)usbCommandSequence,parsed.name,*parsed.arg?",":"",parsed.arg);
    if(n<=0 || n+4>=int(sizeof(frame))){Serial.println("# USB command too long");continue;}
    const char* hex="0123456789ABCDEF";uint8_t checksum=xorChecksum(body);
    frame[0]='$';memcpy(frame+1,body,n);frame[n+1]='*';
    frame[n+2]=hex[checksum>>4];frame[n+3]=hex[checksum&15];frame[n+4]=0;
    handleCommand(frame,now,true);
  }
}
// ---------------- Setup ----------------
void setup() {
  servos.begin(); // drive both signals LOW before startup waits or recovery
  Serial.begin(115200);
  usbCommandSequence=HW_OCOTP_MAC0 ^ ARM_DWT_CYCCNT ^ 0x80000000UL;
  Serial.println("# FSW USB_CMD_2: direct GCS/USB commands supported; send HELP with newline");
  uint32_t t = millis();
  while (!Serial && millis() - t < 1500) {}

  pinMode(PIN_NOR_CS, OUTPUT);
  digitalWrite(PIN_NOR_CS, HIGH);
  pinMode(PIN_LORA_NSS, OUTPUT);digitalWrite(PIN_LORA_NSS,HIGH);

  coexistence.seed(HW_OCOTP_MAC0 ^ HW_OCOTP_MAC1 ^ ARM_DWT_CYCCNT ^ (uint32_t(VEHICLE_ID)<<24));
  SPI.begin();
  resetCause=SRC_SRSR;
  journal.begin(PIN_NOR_CS,millis()); // asynchronous recovery, never erase
  Serial.print("# FLASH JEDEC=0x");Serial.println(flashHardware.detectedId,HEX);
  Wire.begin();   Wire.setClock(I2C_CLOCK_HZ);    // RTC (+ EEPROM later): SDA 18, SCL 19
  Wire1.begin();  Wire1.setClock(I2C_CLOCK_HZ);   // INA219 only
  Wire2.begin();  Wire2.setClock(I2C_CLOCK_HZ);   // BMP581 + BNO085

#if USE_RUNCAM
  Serial5.begin(CAM_BAUD);                        // TX5 = 20, RX5 = 21
#endif

  radioReady=setupLoRa();
  if(!radioReady)Serial.println("# LoRa unavailable: USB commands/sensors/logging remain active; check radio wiring");
  else radioListen();

  rtcOK = setupRTC();
  bmpOK = setupBMP();
  bnoOK = setupBNO();
  setupGPS();
  sdLog.begin('R'); // Teensy onboard microSD slot (SDIO)

  // INA initialization/reads/retries are staggered in loop(), not a boot prerequisite.

  Serial.println("# PAD unarmed: acquiring a stable ground reference; keep rocket still...");
  lastLoop = millis();
  setState(ST_PAD);
}

// ---------------- Main loop ----------------
bool apogeeDetected() {
  if (!flightDataFresh) { apogeeCount = 0; return false; }
  if (acceptedBaro) {
    if (altitude < maxAlt - APOGEE_DROP_M && velocity <= 0)
      { if (apogeeCount < APOGEE_SAMPLES) ++apogeeCount; }
    else apogeeCount = 0;
  }
  return apogeeCount >= APOGEE_SAMPLES;
}

void updatePadReference(uint32_t now, bool gotBaro) {
  if(!recoveryDone || mission.launched || calMask || calibrationPending)return;
  if(baroReferenceReady) {
    // Reacquire current altitude with the SAME reference after a sensor gap/reset.
    if(!vertical.ready && gotBaro) {
      vertical.reset(now,calibration.verticalBias);
      altitude=altPrev=vertical.height=vertical.baroHeight=pressureToAltitude(pressurePa,groundPressure);
      velocity=0;
    }
    return;
  }
  // Altitude availability does not depend on IMU calibration. If a valid IMU
  // detects motion reject the reference; otherwise require stable fresh pressure.
  // ARM and explicit CAL still require the original stationary IMU safety checks.
  if(!baroSeen || uint32_t(now-baroReadMs)>100 || (imuFresh && gyroValid && !calibrationStationary(now))) {
    padStable=false;bootSum=0;bootN=bootAN=0;padAccelSum=0;return;
  }
  if(!padStable && gotBaro){padStable=true;padStableMs=now;padPressureMin=padPressureMax=pressurePa;}
  if(!padStable)return;
  if(gotBaro) {
    padPressureMin=fminf(padPressureMin,pressurePa);padPressureMax=fmaxf(padPressureMax,pressurePa);
    if(fabsf(pressureToAltitude(padPressureMin,padPressureMax))>0.75f) {
      padStable=false;bootSum=0;bootN=bootAN=0;padAccelSum=0;return;
    }
    bootSum+=pressurePa;++bootN;
    if(imuFresh){padAccelSum+=verticalAccel;++bootAN;}
  }
  if(uint32_t(now-padStableMs)>=BOOT_CAL_MS && bootN>=100) {
    FlightCalibration value=calibration;value.pressure=bootSum/bootN;
    if(bootAN>=100)value.verticalBias=padAccelSum/bootAN;
    if(!beginCalibrationWrite(value,false,now))padStable=false;
  }
}

void loop() {
  uint32_t now = millis();
  serviceUsbConsole(now);
  handleLoRa(now);
  serviceWirelessServo();
  serviceRadioSwitch(now);
  readGPS();
  readIMU();
  now = millis();
  updateRTC();
  mission.tick(now);
  if(journal.scanning() && !flashScanStarted){flashScanStarted=true;flashScanStartMs=now;}
  journal.service(now); // one bounded 4 KiB recovery read; normal writes unchanged
  if(flashScanStarted && !journal.scanning() && !flashScanReported){
    flashScanReported=true;
    Serial.printf("# FLASH scan: %lu ms; %lu bytes checked; status=%u\n",
      (unsigned long)(millis()-flashScanStartMs),(unsigned long)journal.scannedBytes(),unsigned(journal.status));
  }
  servicePersistence(now);
  if(!rebootPending && servos.service(journal.committed,journal.usable(),now))
    logEvent(LOG_STATE); // commanded result only; no position feedback
#if USE_INA219
  ina219.service(Wire1,INA_ADDR,now);inaOK=ina219.present;
#endif
  serviceCamera(now); // runs independently of GCS and of the 50 Hz flight block

  serviceReboot(now);
  if (uint32_t(now-lastLoop) < LOOP_PERIOD_MS) {
    if(!rebootPending && !txBusy && !ackPending)sdLog.service(now);
    return;
  }
  float dt = uint32_t(now-lastLoop)/1000.0f;
  lastLoop = now;
  syncRTCFromGNSS();

  imuFresh = accelValid && gravityValid && uint32_t(now-accelMs) <= 100 &&
             uint32_t(now-gravityMs) <= 100 &&
             verticalAcceleration(ax, ay, az, gravityX, gravityY, gravityZ, verticalAccel);
  stationary = imuFresh && gyroValid && uint32_t(now-gyroMs) <= 100 &&
               accelG > 0.85f && accelG < 1.15f && fabsf(verticalAccel-vertical.bias) < 0.5f &&
               fabsf(gx) < 5 && fabsf(gy) < 5 && fabsf(gz) < 5;
  bool gotBaro = bmpOK && readBaro();
  if (gotBaro) { baroReadMs = now; baroSeen = true; }
  if(state==ST_PAD)updatePadReference(now,gotBaro);
  acceptedBaro = vertical.update(now, dt, gotBaro,
                                gotBaro ? pressureToAltitude(pressurePa, groundPressure) : 0,
                                imuFresh, verticalAccel);
  altitude = vertical.height; velocity = vertical.speed; altPrev = altitude;
  flightDataFresh = vertical.fresh(now) && vertical.mode != 0;
  if (acceptedBaro && altitude > maxAlt) maxAlt = altitude;
  runCalibration(now, gotBaro);

  switch (state) {
    case ST_PAD: {
      if(!recoveryDone || recoveryHold || !mission.armed || !armToken || journal.committed<armToken)break;
      if (!vertical.ready || calibrationPending)break;
      if (!flightDataFresh || calMask) { launchCount = fallbackLaunchCount = 0; break; }
      if (acceptedBaro) {
        bool powered = imuFresh && verticalAccel-vertical.bias > (LAUNCH_ACCEL_G-1)*9.80665f;
        launchCount = powered ? (uint8_t)min((int)launchCount+1, LAUNCH_SAMPLES) : 0;
        bool climb = altitude > 15 && velocity > 10;
        fallbackLaunchCount = climb ? (uint8_t)min((int)fallbackLaunchCount+1, LAUNCH_SAMPLES) : 0;
      }
      if ((launchCount >= LAUNCH_SAMPLES && altitude > LAUNCH_MIN_ALT_M && velocity > 0) ||
          fallbackLaunchCount >= LAUNCH_SAMPLES) {
        maxAlt = altitude; maxVel = velocity; setState(ST_ASCENT);
      }
      break;
    }
    case ST_ASCENT:
      if (!flightDataFresh) { burnoutCount = apogeeCount = 0; break; }
      if (velocity > maxVel) maxVel = velocity;
      if (acceptedBaro)
        burnoutCount = velocity < maxVel-BURNOUT_VEL_DROP_MPS ?
          (uint8_t)min((int)burnoutCount+1, BURNOUT_SAMPLES) : 0;
      if (burnoutCount >= BURNOUT_SAMPLES) setState(ST_COAST);
      else if (apogeeDetected()) setState(ST_APOGEE);
      break;
    case ST_COAST:
      if (apogeeDetected()) setState(ST_APOGEE);
      break;
    case ST_APOGEE:
      if (uint32_t(now-stateEnterMs) >= APOGEE_HOLD_MS) setState(ST_DESCENT);
      break;
    case ST_DESCENT: {
      bool quiet = flightDataFresh && stationary && fabsf(altitude) < LANDED_ALT_M &&
                   fabsf(vertical.baroHeight) < LANDED_ALT_M && fabsf(velocity) < LANDED_VEL_MPS &&
                   fabsf(vertical.baroSpeed) < LANDED_VEL_MPS;
      if (!quiet) { landedTimerRunning = false; landingSamples = 0; break; }
      if (!landedTimerRunning) {
        landedTimerRunning = true; landedTimerStart = now; landingSamples = 0;
        landingBaroMin = landingBaroMax = vertical.baroHeight;
      }
      if (acceptedBaro) {
        ++landingSamples;
        landingBaroMin = fminf(landingBaroMin, vertical.baroHeight);
        landingBaroMax = fmaxf(landingBaroMax, vertical.baroHeight);
      }
      if (landingBaroMax-landingBaroMin > 0.5f) { landedTimerRunning = false; landingSamples = 0; }
      else if (uint32_t(now-landedTimerStart) >= LANDED_TIME_MS && landingSamples >= 100)
        setState(ST_LANDED);
      break;
    }
    case ST_LANDED:
      break; // terminal until explicit ARM or RESETPAD; reboot restores mission
  }
}
