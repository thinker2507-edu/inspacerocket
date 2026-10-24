# Onboard SD flight logging

Both `finalfsw` and `cansatfswfinal` now log flight samples to the **Teensy 4.1
onboard microSD slot**, through the Teensy-bundled SD/SdFat libraries and SDIO.
This is not an SPI SD adapter: it uses `BUILTIN_SDCARD`, with no new SPI CS or
changes to the radio, external flash, UART, or sensor pins.

Insert a FAT32 or exFAT microSD before powering the board. Each boot creates a
new file: `R0000000.CSV`, `R0000001.CSV`, ... for rocket; `C0000000.CSV`, ... for
CanSat. File creation is exclusive; existing files are never truncated, appended
to, deleted, or formatted. A maximum of 1000 filenames per vehicle is supported;
archive files on a computer before exhausting that range. Filename checks occur
one per loop, rather than scanning old flight contents at boot.

## What goes where

- **SD:** 5 Hz sensor/flight snapshots, launch/state/landing events, calibration,
  camera intent/results on rocket, packet reservations, reset/recovery events.
  Ordinary sampling is independent of telemetry ON/OFF and flash availability.
- **External W25Q64 flash:** only recovery checkpoints: calibration, packet-number
  reservations, mission/state transitions, camera one-shot intents/results, clock
  changes, and a mission checkpoint every 5 seconds during an incomplete launched
  mission. These preserve the existing recovery behavior. No new routine
  `LOG_SAMPLE` records are written to flash. Historical flash logs are untouched.

Flash startup scanning is still the existing implementation; the proposed fast
startup checkpoint/index is not implemented. A separate explicit USB counter
reset is now documented in [PACKET_COUNTER.md](PACKET_COUNTER.md). SD logging
itself never resets packet numbers.

## Format and integrity

SD files are ordinary text CSV with a header and CRLF-terminated rows, readable
in Excel/LibreOffice. CSV version is **1**; the internal recovery journal remains
binary version 2. `FlightCsv.h` defines all 37 columns and the formatter. Fields
include vehicle, per-file record sequence, readable event name, mission ID,
boot/RTC/mission timestamps, time quality, numeric state, validity flags, relative
altitude/velocity/acceleration, INA voltage/current/power/shunt readings, command
sequence, reset cause, dropped records and calibration generation/biases.

`record_sequence` starts at 1 in each new SD file. It is independent of the LoRa
packet ID and flash journal sequence. `telemetry_reserved_high` is the durable
reservation reference, not the last transmitted packet ID. RTC Unix value zero
means unavailable; `flags` contains the original validity bits. Missing sensor
measurements are `nan`; uncertain elapsed time is `NA`, never an invented zero.
Calibration gyro/vertical biases have separate columns; they are not exported as
accelerometer or INA-voltage measurements. Event codes 6/7 are named
ACTUATOR_INTENT/ACTUATOR_RESULT (the rocket recording requests); these do not prove
physical device state. Numeric state/units retain the existing firmware meanings.

The last column, `row_crc32`, is eight hexadecimal digits: CRC32/IEEE of the ASCII
row bytes **before the comma introducing the CRC column**, excluding that comma,
the CRC itself and CRLF. The header has no CRC. A reader can ignore an incomplete
final line or reject a line whose CRC fails. CSV row integrity does not guarantee
that filesystem metadata survives power loss. This is the richer onboard event
CSV schema, not the GCS live/replay CSV schema. Old BIN logs are untouched and are
not automatically converted.

The RAM queue holds 16 records, reserves four places for events, and can evict a
queued sample for a critical event. Drops are counted. Data is synced at least
once per second while dirty, or on the next service call after an event write.
A reset/power loss can still lose buffered records or damage FAT metadata; CRC
does not make a consumer SD filesystem power-loss proof. Critical flight recovery
therefore continues to use the separately committed external-flash journal.

## Status and failures

USB Serial Monitor at 115200 prints SD initialization/file/failure messages.
The existing GCS command console can send **`STATUS,SD`** to the selected vehicle.
The authenticated existing ACK returns:

`SD=<state>;FILE=<name>;WR=<records written>;DROP=<dropped>;SLOW=<slow operations>`

States: 0 missing/uninitialized, 1 finding a new filename, 2 ready, 3 failed, 4 closed for reboot.
`WR` means records accepted by the filesystem, not necessarily synced. `SLOW`
counts service write/sync calls exceeding 20 ms. Initialization/file-opening
latency is not included. A full card, write/sync error or prolonged busy state
fails the logger; it does not retry indefinitely or fall back to flash samples.
There is no automatic hot-insert/remount: insert the card before boot. Sensor,
radio, flight-state and flash recovery paths are not disabled by an SD error.

Only one file write or sync is attempted per service call, when no radio TX/ACK
is pending and the next 50 Hz flight update is not yet due. SD calls are still
synchronous and can occasionally exceed that remaining time. No hard real-time
bound, actual card durability, or physical hardware validation is claimed.

## Build/use

Open the existing `finalfsw/finalfsw.ino` or
`cansatfswfinal/cansatfswfinal.ino` in the Teensy Arduino environment, select
Teensy 4.1 and use its bundled SD/SdFat libraries. No receiver/GCS update is
required. Confirm `SD=2` and an increasing `WR` with `STATUS,SD`. Switch telemetry
OFF on the pad and confirm `WR` still increases. Stop testing and power down
before removing the card, then copy the new CSV file to a computer.

## Software reboot

Send `REBOOT` through the GCS command console, or directly to the flight Teensy
USB Serial Monitor at 115200 with Newline enabled. USB also accepts lowercase
`reboot`. Both use the same handler; radio authentication/checksum rules remain.
Real flight firmware requires completed startup recovery, **unarmed PAD**, no
launched mission, and no calibration/camera operation pending. Rejections state
the reason. The Teensy programming button is not a software restart command.

Routine radio transmissions and new log snapshots stop once reboot is accepted;
the command ACK is given priority before shutdown. Existing flash writes and SD
rows drain, the CSV file is synced and closed, and the Cortex-M7 SYSRESETREQ is
issued after a short non-blocking message grace period. Storage gets a 2-second
best-effort drain window; failed storage does not prevent reboot indefinitely.
If it times out, USB reports that unsynced rows may be lost. SD library calls can
still stall beyond that scheduling deadline. A normal reboot creates the next
CSV file and restores existing mission/calibration/packet reservations. It never
resets the packet count or erases log history.

For the ESP32 fake CanSat, `REBOOT` now works over its USB console and authenticated
GCS path too; it uses `esp_restart()` after ACK/TX completion (bounded failure
fallback) and preserves its NVS packet reservation. It has no SD logger.