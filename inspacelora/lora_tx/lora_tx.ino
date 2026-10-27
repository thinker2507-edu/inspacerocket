
#include <RadioLib.h>
#include <SPI.h>

// ---------- Radio configuration (MUST match receiver) ----------
#define FREQ_MHZ   865.5      // choose 865.5 or 867.25
// #define FREQ_MHZ 867.25
#define BW_KHZ     125.0
#define SF         7
#define CR         5          // coding rate 4/5
#define SYNC_WORD  0x95
#define TX_POWER   14         // dBm

// ---------- Pins ----------
#define PIN_SCK    18
#define PIN_MISO   19
#define PIN_MOSI   23
#define PIN_NSS    5
#define PIN_DIO1   26
#define PIN_RESET  25
#define PIN_BUSY   27

SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RESET, PIN_BUSY);

// ---------- Telemetry variables (default values) ----------
uint16_t teamId          = 1000;             // <TEAM ID>
uint32_t timeStamp       = 0;                // <TIME STAMPING> (ms since boot)
uint32_t packetCount     = 0;                // <PACKET COUNT>
float    altitude        = 0.0;              // <ALTITUDE> (m)
float    pressure        = 0.0;              // <PRESSURE> (Pa)
float    temperature     = 0.0;              // <TEMP> (deg C)
float    voltage         = 0.0;              // <VOLTAGE> (V)
char     gnssTime[12]    = "00:00:00";       // <GNSS TIME> (hh:mm:ss)
float    gnssLat         = 0.0;              // <GNSS LATITUDE>
float    gnssLon         = 0.0;              // <GNSS LONGITUDE>
float    gnssAlt         = 0.0;              // <GNSS ALTITUDE> (m)
uint8_t  gnssSats        = 0;                // <GNSS SATS>
float    accel[3]        = {0.0, 0.0, 0.0};  // <ACCELEROMETER DATA> x,y,z
float    gyroSpin[3]        = {0.0, 0.0, 0.0};  // <GYRO SPIN RATE> (deg/s)
uint8_t  flightState     = 0;                // <FLIGHT SOFTWARE STATE>
char     optionalData[32] = "NONE";          // <OPTIONAL DATA> (no commas!)

void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  int16_t state = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR, SYNC_WORD, TX_POWER, 8);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Radio init failed, code: ");
    Serial.println(state);
    while (true) delay(1000);
  }

  radio.setTCXO(1.8);              // only if your module has a TCXO
  radio.setDio2AsRfSwitch(true);   // only if DIO2 drives the RF switch
  // E22 modules instead: radio.setRfSwitchPins(RXEN_PIN, TXEN_PIN);

  Serial.println("LoRa transmitter ready");
}

// Build the 16-field CSV packet
int buildPacket(char *buf, size_t len) {
  return snprintf(buf, len,
    "%u,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%s,%.6f,%.6f,%.2f,%u,%.2f;%.2f;%.2f,%.2f,%u,%s",
    teamId,
    (unsigned long)timeStamp,
    (unsigned long)packetCount,
    altitude,
    pressure,
    temperature,
    voltage,
    gnssTime,
    gnssLat,
    gnssLon,
    gnssAlt,
    gnssSats,
    accel[0], accel[1], accel[2],
    gyroSpin,
    flightState,
    optionalData);
}

void loop() {
  timeStamp = millis();
  packetCount++;

  // TODO: update altitude, pressure, temperature, voltage, GNSS data,
  // accel[], gyroSpin and flightState from your sensors here.

  char packet[200];
  int n = buildPacket(packet, sizeof(packet));
  if (n <= 0 || n >= (int)sizeof(packet)) {
    Serial.println("Packet too long");
    return;
  }

  int16_t state = radio.transmit((uint8_t*)packet, n);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("Sent ("); Serial.print(n); Serial.print(" bytes): ");
    Serial.println(packet);
  } else {
    Serial.print("TX failed, code: ");
    Serial.println(state);
  }

  delay(1000);   // 1 Hz
}
