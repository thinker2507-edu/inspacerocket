
#include <RadioLib.h>
#include <SPI.h>
#include "../../finalfsw/CommandValidation.h"
#include "../../finalfsw/ManualRadioSwitch.h"

// Output mode:
//   1 = machine bridge: unchanged radio payload + newline (including ACKs)
//   0 = human-readable labelled output
#define CSV_ONLY   1

// ---------- Radio configuration (MUST match transmitter) ----------
#define FREQ_MHZ   865.5      // choose 865.5 or 867.25
// #define FREQ_MHZ 867.25
#define BW_KHZ     125.0
#define SF         7
#define CR         5
#define SYNC_WORD  0x95
#define TX_POWER   14

// ---------- Pins ----------
#define PIN_SCK    18
#define PIN_MISO   19
#define PIN_MOSI   23
#define PIN_NSS    5
#define PIN_DIO1   26
#define PIN_RESET  25
#define PIN_BUSY   27

SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RESET, PIN_BUSY);

// ---------- Received telemetry variables ----------
uint16_t teamId;
uint32_t timeStamp;
uint32_t packetCount;
float    altitude, pressure, temperature, voltage;
char     gnssTime[12];
float    gnssLat, gnssLon, gnssAlt;
uint8_t  gnssSats;
float    accel[3];
float    gyroSpin;
uint8_t  flightState;
char     optionalData[32];

uint32_t lastCount[2] = {};
uint32_t missedTotal[2] = {};
char vehicleId=0;

volatile bool rxFlag = false;
void IRAM_ATTR onRx() { rxFlag = true; }

// One bounded host command at a time. No retransmission of button operations.
char commandLine[100];
size_t commandUsed = 0;
bool commandDropping = false, commandReady = false;
uint32_t commandLastByteMs = 0, commandDueMs = 0;
uint32_t commandQueuedMs=0,uplinkTxStart=0,uplinkTxWait=0;
uint32_t lastDownlinkMs[2]={}; // R, C: never borrow the other vehicle's RX window
bool haveDownlink[2]={},uplinkTxBusy=false,waitDownlink=false;
uint8_t commandTarget=0;
bool groundHeaderActive=false;
uint32_t groundHeaderMs=0;

uint32_t groundRadioIrq(){
#if defined(RADIOLIB_VERSION_MAJOR) && RADIOLIB_VERSION_MAJOR>=7
  return radio.getIrqFlags();
#else
  return radio.getIrqStatus();
#endif
}

bool validUplink(const char* s, size_t n) {
  return commandAddressMatches(s,n,23,'R') || commandAddressMatches(s,n,23,'C');
}

RadioChannel receiverChannel=RadioChannel::MAIN;
RadioSwitchRequest receiverSwitchRequest;
bool awaitSwitchAck=false;
uint32_t switchRequestMs=0;

bool retuneReceiver(RadioChannel channel){
  const RadioChannel previous=receiverChannel;
  int16_t rc=radio.standby();rxFlag=false;groundHeaderActive=false;
  if(rc==RADIOLIB_ERR_NONE)rc=radio.setFrequency(radioChannelMHz(channel));
  if(rc==RADIOLIB_ERR_NONE)rc=radio.startReceive();
  if(rc==RADIOLIB_ERR_NONE)receiverChannel=channel;
  else {radio.setFrequency(radioChannelMHz(previous));radio.startReceive();}
  haveDownlink[0]=haveDownlink[1]=false;
  Serial.printf("# RADIO RECEIVER=%s;MHZ=%.2f;SF=7;SYNC=0x95;RC=%d;LINK=UNCONFIRMED\n",radioChannelName(receiverChannel),radioChannelMHz(receiverChannel),rc);
  return rc==RADIOLIB_ERR_NONE;
}
void receiverRadioConsole(const char* arg){
  if(!strcmp(arg,"STATUS")){
    Serial.printf("# RADIO RECEIVER=%s;MHZ=%.2f;SF=7;SYNC=0x95;LINK=UNCONFIRMED\n",radioChannelName(receiverChannel),radioChannelMHz(receiverChannel));return;
  }
  RadioChannel channel=radioChannel(arg);
  if(channel==RadioChannel::INVALID){Serial.println("# RADIO invalid receiver argument: MAIN BACKUP STATUS");return;}
  awaitSwitchAck=false;retuneReceiver(channel); // manual local recovery; no RF command
}
void observeRadioSwitchAck(const char* frame,size_t length){
  if(!awaitSwitchAck || !radioReplyMatches(frame,length,receiverSwitchRequest))return;
  const bool accepted=acceptedRadioSwitch(frame,length,receiverSwitchRequest);
  awaitSwitchAck=false;
  if(accepted)retuneReceiver(receiverSwitchRequest.channel);
}
void serviceUplink() {
  uint32_t now = millis();
  if(uplinkTxBusy){
    if(uint32_t(now-uplinkTxStart)<uplinkTxWait)return;
    radio.finishTransmit();uplinkTxBusy=false;rxFlag=false;
    int16_t rc=radio.startReceive();
    if(rc!=RADIOLIB_ERR_NONE){Serial.print("# RX rearm failed: ");Serial.println(rc);}
    return;
  }
  if(awaitSwitchAck && uint32_t(now-switchRequestMs)>6000){
    awaitSwitchAck=false;Serial.println("# RADIO switch ACK not received; outcome UNKNOWN; use GROUND MAIN/BACKUP to recover receiver");
  }
  if (commandUsed && now - commandLastByteMs > 1000) {
    commandUsed = 0; commandDropping = true; // discard through next LF
    Serial.println("# UPLINK partial line timed out");
  }
  while (!commandReady && Serial.available()) {
    char c = (char)Serial.read();
    commandLastByteMs = now;
    if (c == '\n') {
      commandLine[commandUsed] = 0;
      if(!commandDropping && !strncmp(commandLine,"@RADIO ",7)){
        receiverRadioConsole(commandLine+7);commandUsed=0;commandDropping=false;continue;
      }
      if (!commandDropping && validUplink(commandLine, commandUsed)) {
        commandReady = true;
        commandTarget=commandAddressMatches(commandLine,commandUsed,23,'C')?1:0;
        commandQueuedMs=now;
        waitDownlink=!haveDownlink[commandTarget] || uint32_t(now-lastDownlinkMs[commandTarget])>150;
        commandDueMs=now+50;
      } else if (commandUsed || commandDropping) Serial.println("# UPLINK invalid or oversized line");
      commandUsed = 0; commandDropping = false;
    } else if (!commandDropping) {
      // GCS sends LF only. A CR or non-ASCII byte invalidates the whole line.
      if ((c < 33 && !receiverRadioSeparator(commandLine,commandUsed,c)) || c > 126 || commandUsed >= sizeof(commandLine) - 1) commandDropping = true;
      else commandLine[commandUsed++] = c;
    }
  }
  if(!commandReady || rxFlag || (int32_t)(now-commandDueMs)<0)return;
  // Prefer an observed receive window. With downlink disabled, a bounded idle
  // wait still permits TELEM,1. No repeat transmission is scheduled.
  if(waitDownlink && uint32_t(now-commandQueuedMs)<1200)return;
  if(groundRadioIrq()&RADIOLIB_SX126X_IRQ_HEADER_VALID){
    if(!groundHeaderActive){groundHeaderActive=true;groundHeaderMs=now;}
    if(uint32_t(now-groundHeaderMs)<500)return;
    radio.standby();rxFlag=false;radio.startReceive();
    groundHeaderActive=false;commandDueMs=now+50;return;
  }
  groundHeaderActive=false;
  radio.standby();
  rxFlag = false;
  size_t n=strlen(commandLine);
  int16_t state = radio.startTransmit((uint8_t*)commandLine,n);
  Serial.print("# UPLINK radio TX start (not execution): "); Serial.println(state);
  commandReady = false;
  if(state==RADIOLIB_ERR_NONE){
    if(parseRadioSwitchRequest(commandLine,n,receiverSwitchRequest)){awaitSwitchAck=true;switchRequestMs=now;}
    uplinkTxBusy=true;uplinkTxStart=now;uplinkTxWait=radio.getTimeOnAir(n)/1000+30;return;
  }
  rxFlag = false;
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) { Serial.print("# RX rearm failed: "); Serial.println(state); }
}

// Parse CSV line (modifies the buffer) into variables
bool parsePacket(char *line) {
  if(line[0]=='$')return false; // ACK/SYS have their own schemas
  vehicleId=fleetDownlinkVehicle(line,strlen(line),23);
  if(!vehicleId)return false;
  *strrchr(line,'*')=0;
  char *fields[16];
  int count = 0;

  char *p = line;
  fields[count++] = p;
  while (*p && count < 16) {
    if (*p == ',') {
      *p = '\0';
      fields[count++] = p + 1;
    }
    p++;
  }
  if (count != 16) return false;

  teamId      = atoi(fields[0]);
  timeStamp   = strtoul(fields[1], NULL, 10);
  packetCount = strtoul(fields[2], NULL, 10);
  altitude    = atof(fields[3]);
  pressure    = atof(fields[4]);
  temperature = atof(fields[5]);
  voltage     = atof(fields[6]);
  strncpy(gnssTime, fields[7], sizeof(gnssTime) - 1);
  gnssTime[sizeof(gnssTime) - 1] = '\0';
  gnssLat     = atof(fields[8]);
  gnssLon     = atof(fields[9]);
  gnssAlt     = atof(fields[10]);
  gnssSats    = atoi(fields[11]);

  if (sscanf(fields[12], "%f;%f;%f", &accel[0], &accel[1], &accel[2]) != 3)
    return false;

  gyroSpin    = atof(fields[13]);
  flightState = atoi(fields[14]);
  strncpy(optionalData, fields[15], sizeof(optionalData) - 1);
  optionalData[sizeof(optionalData) - 1] = '\0';
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  int16_t state = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR, SYNC_WORD, TX_POWER, 8);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("# Radio init failed, code: ");
    Serial.println(state);
    while (true) delay(1000);
  }

  radio.setTCXO(1.8);              // only if your module has a TCXO
  radio.setDio2AsRfSwitch(true);   // only if DIO2 drives the RF switch
  // E22 modules instead: radio.setRfSwitchPins(RXEN_PIN, TXEN_PIN);

  radio.setPacketReceivedAction(onRx);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("# startReceive failed, code: ");
    Serial.println(state);
    while (true) delay(1000);
  }

#if !CSV_ONLY
  Serial.println("Receiver ready, waiting for packets...");
#endif
}

void loop() {
  if(uplinkTxBusy || !rxFlag){serviceUplink();return;}
  rxFlag = false;

  char buf[256];
  size_t len = radio.getPacketLength();
  if (len == 0 || len >= sizeof(buf)) { radio.startReceive(); return; }
  int16_t state = radio.readData((uint8_t*)buf, len);

  if (state == RADIOLIB_ERR_NONE) {
    buf[len] = '\0';
    char vehicle=fleetDownlinkVehicle(buf,len,23);
    if(vehicle){
      uint8_t target=vehicle=='C'?1:0;
      lastDownlinkMs[target]=millis();haveDownlink[target]=true;
      if(commandReady && target==commandTarget){
        waitDownlink=false;commandDueMs=lastDownlinkMs[target]+50;
      }
    }
    float rssi = radio.getRSSI();
    float snr  = radio.getSNR();
    observeRadioSwitchAck(buf,len); // exact matching checksum/address/sequence only

#if CSV_ONLY
    Serial.write((uint8_t*)buf, len); Serial.write('\n');
#else
    Serial.print("RAW: "); Serial.println(buf);

    if (parsePacket(buf)) {
      uint8_t target=vehicleId=='C'?1:0;
      if (lastCount[target] != 0 && packetCount > lastCount[target] + 1)
        missedTotal[target] += packetCount - lastCount[target] - 1;
      lastCount[target] = packetCount;

      Serial.println("---------------------------------");
      Serial.printf("Team ID       : %u\n", teamId);
      Serial.printf("Vehicle       : %c\n", vehicleId);
      Serial.printf("Time stamp    : %lu ms\n", (unsigned long)timeStamp);
      Serial.printf("Packet count  : %lu  (missed so far: %lu)\n",
                    (unsigned long)packetCount, (unsigned long)missedTotal[target]);
      Serial.printf("Altitude      : %.2f m\n", altitude);
      Serial.printf("Pressure      : %.2f Pa\n", pressure);
      Serial.printf("Temperature   : %.2f C\n", temperature);
      Serial.printf("Voltage       : %.2f V\n", voltage);
      Serial.printf("GNSS time     : %s\n", gnssTime);
      Serial.printf("GNSS lat/lon  : %.6f, %.6f\n", gnssLat, gnssLon);
      Serial.printf("GNSS altitude : %.2f m\n", gnssAlt);
      Serial.printf("GNSS sats     : %u\n", gnssSats);
      Serial.printf("Accel x,y,z   : %.2f, %.2f, %.2f\n", accel[0], accel[1], accel[2]);
      Serial.printf("Gyro spin     : %.2f deg/s\n", gyroSpin);
      Serial.printf("Flight state  : %u\n", flightState);
      Serial.printf("Optional      : %s\n", optionalData);
      Serial.printf("RSSI / SNR    : %.0f dBm / %.1f dB\n", rssi, snr);
    } else {
      Serial.println("Parse error (wrong field count or format)");
    }
#endif
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
#if !CSV_ONLY
    Serial.println("CRC error (corrupted packet)");
#endif
  } else {
#if !CSV_ONLY
    Serial.print("Read failed, code: ");
    Serial.println(state);
#endif
  }

  radio.startReceive();   // back to listening
}
