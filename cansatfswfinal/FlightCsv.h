#pragma once
#include <stdio.h>
#include "FlightJournal.h"

constexpr const char* FLIGHT_CSV_HEADER=
  "csv_version,vehicle,record_sequence,event,mission,boot_ms,rtc_unix,launch_rtc_unix,"
  "mission_elapsed_ms,time_quality,state,flags,ground_pressure_pa,altitude_m,velocity_m_s,"
  "max_altitude_m,max_velocity_m_s,accel_x_m_s2,accel_y_m_s2,accel_z_m_s2,bus_voltage_v,"
  "shunt_mv,current_ma,power_mw,command_sequence,landing_rtc_unix,landing_elapsed_ms,"
  "reset_cause,dropped,detail,telemetry_reserved_high,calibration_generation,"
  "gyro_bias_x_deg_s,gyro_bias_y_deg_s,gyro_bias_z_deg_s,vertical_bias_m_s2,row_crc32\r\n";
constexpr const char* flightEventName(uint8_t type){
  switch(type){
    case LOG_RESET:return "RESET";case LOG_ARM:return "ARM";case LOG_LAUNCH:return "LAUNCH";
    case LOG_STATE:return "STATE";case LOG_SAMPLE:return "SAMPLE";
    case 6:return "ACTUATOR_INTENT";case 7:return "ACTUATOR_RESULT";
    case LOG_COMPLETE:return "COMPLETE";case LOG_RECOVERY:return "RECOVERY";
    case LOG_CLOCK:return "CLOCK_CHECKPOINT";case LOG_CALIBRATION:return "CALIBRATION";
    case LOG_PACKET_RANGE:return "PACKET_RESERVATION";case LOG_COUNTER_RESET:return "COUNTER_RESET";
    case LOG_MISSION_RESET:return "MISSION_RESET";
    default:return "UNKNOWN";
  }
}
inline int formatFlightCsv(char* out,size_t capacity,const FlightRecord& r,char vehicle){
  char elapsed[24],landing[24];
  if(r.quality==TIME_UNCERTAIN || r.elapsed==UNKNOWN_ELAPSED)snprintf(elapsed,sizeof(elapsed),"NA");
  else snprintf(elapsed,sizeof(elapsed),"%llu",(unsigned long long)r.elapsed);
  if(r.landingElapsed==UNKNOWN_ELAPSED)snprintf(landing,sizeof(landing),"NA");
  else snprintf(landing,sizeof(landing),"%llu",(unsigned long long)r.landingElapsed);
  bool cal=r.type==LOG_CALIBRATION;
  // Calibration records repurpose ax/ay/az/voltage for biases in the internal
  // checkpoint layout. Export them in dedicated columns, never as measurements.
  int n=snprintf(out,capacity,
    "1,%c,%lu,%s,%lu,%lu,%lu,%lu,%s,%u,%u,%lu,%.8g,%.8g,%.8g,%.8g,%.8g,"
    "%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%lu,%lu,%s,%lu,%lu,%u,%lu,%lu,%.8g,%.8g,%.8g,%.8g",
    vehicle,(unsigned long)r.sequence,flightEventName(r.type),(unsigned long)r.mission,
    (unsigned long)r.bootMs,(unsigned long)r.rtcUnix,(unsigned long)r.launchRtc,elapsed,
    unsigned(r.quality),unsigned(r.state),(unsigned long)r.flags,r.groundPressure,r.altitude,r.velocity,
    r.maxAltitude,r.maxVelocity,cal?NAN:r.ax,cal?NAN:r.ay,cal?NAN:r.az,cal?NAN:r.voltage,
    r.shunt_mV,r.current_mA,r.power_mW,(unsigned long)r.commandSequence,(unsigned long)r.landingRtc,
    landing,(unsigned long)r.resetCause,(unsigned long)r.dropped,unsigned(r.metadata.detail),
    (unsigned long)r.metadata.telemetryHigh,(unsigned long)r.metadata.calibrationGeneration,
    cal?r.ax:NAN,cal?r.ay:NAN,cal?r.az:NAN,cal?r.voltage:NAN);
  if(n<=0 || size_t(n)>=capacity)return -1;
  uint32_t crc=flightCrcBytes((const uint8_t*)out,n);
  int tail=snprintf(out+n,capacity-n,",%08lX\r\n",(unsigned long)crc);
  return tail>0 && size_t(n+tail)<capacity?n+tail:-1;
}
