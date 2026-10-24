# CanSat flight target

Flight samples now go to the onboard microSD slot using Teensy's bundled SD/SdFat
libraries, while external flash retains recovery checkpoints. Insert a FAT32 or
exFAT card before boot and query `STATUS,SD`. See [SD_LOGGING.md](../SD_LOGGING.md).


Open **cansatfswfinal.ino** in Arduino IDE, select **Teensy 4.1**, and build/upload
with the same installed libraries as the rocket: RadioLib, SparkFun BMP581,
Adafruit BNO08x/BusIO/Unified Sensor, TinyGPSPlus, RTClib and the Teensy core's
SerialFlash. All project headers are local to this folder; no rocket source is
included. Open **gcs.html** for this target. The root rocket GCS is unchanged in
purpose and retains its recording controls.

Both targets now use local `FlightImu.h` / `ImuSample.h` adapters to correct the
installed I2C driver's missing receive timestamp and preserve all reports within
a bundled packet. Fresh axes are distinct from reliable calibration quality;
flight/calibration checks still require reliable IMU data. Telemetry copies the
decoded axes directly as in the RTC-final reference and holds the last readings;
it does not gate them on IMU state, accuracy or sample age. Stored gyro offsets
remain applied. No IMU-state query is needed for normal display. Use **STATUS,IMU** in
the GCS console for freshness/accuracy/configuration/reset diagnostics. See
REPAIR_NOTES.md for the source trace and limitations. Both GCS phase bars now
map states 0..5 exactly, including COAST; there is no probe-release flight phase.

Startup is **PAD, unarmed**, with no separate BOOT state. Sensor/reference
initialization continues in PAD. Explicit ARM and its committed journal record
are still required before launch detection; reset still recovers interrupted
missions. TELEM,1 is not restricted by flight state or arming. Direct Teensy USB
command input is not implemented: the existing GCS commands use the ground LoRa
receiver, while Teensy USB prints routine packets after successful radio TX.

Flash recovery now reads/checks one **4 KiB chunk per loop pass**, needing 2,048
passes for 8 MiB. It retains the full-region integrity scan and all existing
mission/calibration/counter recovery rules. USB prints measured scan completion
time and status. Actual hardware timing has not yet been verified.

The existing **../inspacelora/lora_rx/lora_rx.ino** ground firmware is shared:
it forwards raw downlinks and authenticated command frames without interpreting
the sensor schema. Use the updated vehicle-address-aware receiver. No different
ground hardware or radio configuration is needed.
Keep the repository together to build that receiver's shared command validator.

## Hardware retained from the inspected baseline

| Device | Interface / pins | Configuration |
| --- | --- | --- |
| SX1262 | SPI MOSI11/MISO12/SCK13, NSS2, RESET6, BUSY4, RXEN1/TXEN0 | Existing 865.5 MHz / BW125 / SF7 / CR5 / sync95; unchanged |
| BMP581 | Wire2 SDA25/SCL24 | 0x47, fallback 0x46; 50 Hz; Pa / degrees C |
| BNO085 | Wire2 SDA25/SCL24 | 0x4A, fallback 0x4B; existing gravity/orientation processing |
| DS3231 | Wire SDA18/SCL19 | 0x68; checked UTC validity; no boot time reset |
| INA219 | Wire1 SDA17/SCL16 | 0x40; existing voltage/current/power calibration |
| MAX-M10S GNSS | Serial7 RX28/TX29 | Existing baud detection and configuration |
| W25Q64JV | SPI11/12/13, separate CS10 | Expected JEDEC EF4017, 8 MiB, SerialFlash; 3.3 V wiring requires physical confirmation |
| PSMS | **Unknown** | No sensor-specific electrical or protocol assumptions |

No camera UART is initialized, and no camera packets, commands, lifecycle flags
or automatic recording-stop timer exist in this target. Mission completion is
committed after the existing confirmed landing transition, independent of any
camera. Flight-state detection, gravity-projected complementary estimator,
calibration, launch-only mission clock and interrupted-mission recovery remain
the baseline implementations.

## PSMS boundary and missing information

`PsmsSensor.h` is deliberately UNCONFIGURED. `begin()` and `service()` perform no
hardware access and cannot block another subsystem. Driver status 0 is included
as `PSMS=0` in STATUS acknowledgements and in journal `metadata.detail` for flight
snapshots. The GCS explicitly displays UNCONFIGURED. There are **no PSMS
measurements** and no claimed measurement units or validity until hardware is
identified. The legacy aerosol chart remains empty; its old units are not used
as evidence of a sensor specification.

Needed before implementing a real driver: manufacturer and exact model; supply
voltage/current and logic levels; UART/I2C/SPI/analog interface; all signal/power
wiring (including enable/reset); baud/address/bus settings; frame layout,
checksum, measurement names/units/scaling, validity/warm-up rules and sample rate;
any required calibration and vendor library/documentation. No matching PSMS
model, sketch or schematic was found in the supplied files. Do not connect it to
the old camera UART on that assumption.

Once known, implement bounded acquisition in this driver and update CanSat-only
telemetry/parser and versioned records together. No PSMS commands are invented.

## Protocol and storage differences

- Team 23, key 4821, field counts, XOR and radio settings are retained. The address
  field is now **23:C**, distinct from rocket **23:R**, in commands/ACK/SYS and
  telemetry. Unaddressed legacy live frames are rejected. The CanSat GCS blocks
  CAM/CAMPWR, including console envelopes. The
  firmware has no handlers for those names. Use the correct GCS file. These
  vehicle addresses isolate the two boards. The common RF channel can still
  collide: bounded backoff/peer quiet windows mitigate this without guaranteeing
  delivery. See **../INTERFERENCE_NOTES.md** for the protocol and coexistence limits.
- Sensor telemetry still has 16 fields and the same units/order, with address
  23:C in field 0 and a new trailing `*XX` XOR over the CSV body. Field 14 remains
  numeric flight state. Field 15 is **2**, the existing GCS CanSat source marker,
  matching rocket source marker 1. No `;measurement` suffix is sent because PSMS
  measurements/units are unknown. The updated ground bridge forwards both sources;
  each GCS now accepts only its selected vehicle. The dedicated GCS updates primary map/phase panels
  from CanSat data. The SYS reserved field 13 is 0; it has no camera meaning here.
- Records remain version 2, 128 bytes, CRC32 and separate A5 commit. The CanSat
  signature is **CFL1**, distinct from rocket **RFL1**. Previously used rocket
  flash is reported FOREIGN and never erased or reused automatically. Use a
  dedicated empty flash device for a new CanSat deployment; archive existing
  records before any separately authorized erase. Record IDs 6/7 and old camera
  flag bits are reserved, not repurposed as sensor readings.
- The independent journal retains append-only recovery, calibration records,
  64-number durable reservations, damaged-slot range burning, explicit full/fault
  handling and counter exhaustion. Boot skips unused reserved numbers. Launch
  alone starts mission time; reset restores the last flight record and trusted,
  advancing RTC time, otherwise reports time uncertainty. Reset never rearms.

## Telemetry control and use

Connect the ground receiver in the CanSat GCS and wait for STATUS/SYS state.
Calibrate while stationary, wait for DONE, establish UTC, then Arm Mission.
Telemetry OFF/ON sends explicit TELEM,0/1, once, without retries. ACKs and essential
health/mission diagnostics continue while OFF; periodic sensor packets and INA
measurement values in SYS do not. Sensors, 5 Hz flash logging and flight timing
continue. ON constructs fresh packets, without resetting any mission or counter.

Preserved boot/safety policy: boot starts telemetry ON; launch forces ON; OFF
during flight is rejected as TELEM_REQUIRED_IN_FLIGHT. OFF is allowed before
launch and after landing. Reconnect queries STATUS, never assumes a default.
During journal recovery, OFF is rejected as RECOVERY_SCANNING until the actual
flight safety state is known.
Requested, last-confirmed and currently-known state are shown separately.

Run `powershell -NoProfile -ExecutionPolicy Bypass -File ../tests/verify.ps1` from
this folder (or `tests/verify.ps1` from the root). Checks cover both targets:
30-second OFF scheduling simulation, ACK/health retention, ON/idempotence,
GCS syntax and handlers, reservation/torn-record/mission recovery, and ARM object
compilation with flash enabled and disabled. These do not link uploadable images
or validate physical sensors, radio, PSMS, flash wiring or power-loss behavior.
Upgrade both flight targets, the receiver and GCS together for the addressed
protocol; no flash erase or packet-counter reset is needed. Additional focused
checks cover cross-vehicle traffic, command isolation and bounded coexistence.
