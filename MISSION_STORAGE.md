# Flight mission storage and operation

> USB command update (USB_CMD_2): the flight Teensy now accepts all ordinary GCS
> commands directly over USB, in either NAME,arg or NAME arg form. Plain text and
> authenticated GCS frames reach the same command handler; replies include final
> DONE/FAIL on the originating connection. STATUS LINK identifies this build.
> RESETCOUNT remains local USB-only. See REPAIR_NOTES.md for flashing/use steps.


## Explicit reset to a fresh PAD mission

Flash the updated target and open its updated GCS. Send **`RESETPAD,CONFIRM`**
through the selected vehicle's GCS command console, or **`RESETPAD CONFIRM`** to
the flight board's USB Serial Monitor (115200, Newline). This is distinct from
REBOOT and RESETCOUNT. No receiver change or new command framing is required.

Real flight firmware allows this from PAD (including armed PAD) or LANDED, after
startup recovery with an idle, healthy flash journal. Ascent/coast/apogee/descent
and an ambiguous active mission on PAD are rejected. Stop the requested recording
first on rocket; camera/calibration/counter operations must not be pending. All
rejection reasons are returned explicitly. A transient storage-busy result can
be retried manually once the pending operation finishes; there is no auto retry.

The command immediately disarms launch detection, queues a critical
`LOG_MISSION_RESET` record (type 14), and responds `OK,PAD_RESET_PERSISTING`.
Only after the record commits does it apply the new unarmed PAD mission and send
`DONE,ST=0;PAD_RESET_DONE;UNARMED;MISSION=<new ID>`. USB prints the corresponding
result; GCS waits for DONE/FAIL and updates its phase marker and mission timer.
If writing fails or times out, it reports failure, holds the mission unarmed and
freezes persistence until reboot recovery resolves any ambiguous commit.

Reset clears the mission clock/launch and landing references, maxima and phase
counters, arming token, estimator initialization, recovery hold, and rocket's
landing-to-stop timer and old recording-request lifecycle. It sends no camera
UART command and does not assert a physical camera state. The normal stable-pad
reference acquisition runs again; the mission stays unarmed until the existing
ARM workflow is intentionally used. Sensor drivers, RTC/GNSS/INA, saved calibration
records, telemetry ON/OFF/rate, radio settings, packet IDs/epoch and existing SD
files/flash history are retained. Pad stabilization updates its ground reference
through the existing calibration routine, rather than zeroing raw sensor axes.
SD logs include a MISSION_RESET event; normal logging continues in the same file.

A committed reset restores the new unstarted PAD mission after power cycling. A
torn reset preserves the prior valid mission. Calibration and packet reservations
remain independent. Do not downgrade to a reader that predates type-14 records
and expect that new reset checkpoint to be interpreted correctly.

The ESP32 fake CanSat accepts the same command without pretending to persist a
real mission: it holds its synthetic profile in PAD until USB `FLIGHT` restarts
the cycle. Its NVS telemetry counter is not reset. It has no physical camera/SD.


> Storage update: routine flight logs now use the Teensy 4.1 onboard microSD.
> External flash retains only recovery checkpoints; existing flash history is
> preserved. See [SD_LOGGING.md](SD_LOGGING.md). Descriptions below of 5 Hz flash
> samples describe the earlier implementation and are superseded by that update.


Current repair details and legacy packet-counter migration: [REPAIR_NOTES.md](REPAIR_NOTES.md).

Recovery now scans in bounded **4 KiB / 32-record chunks**, one chunk per loop
pass: **2,048 reads for the 8 MiB device**, rather than one pass per 128-byte slot.
It still verifies the entire device and preserves records beyond erased gaps,
CRC/commit checks, calibration and sequence reservations. No erase or migration is
needed. USB reports scan elapsed time, bytes checked and final storage status;
elapsed hardware performance remains to be measured. Writes retain their existing
one-page body/commit scheme. See the faster-recovery notes in REPAIR_NOTES.md.

## Hardware found / configuration

The supplied marking `25Q64jvs10` identifies the W25Q64JV family. The logger is
enabled for **8 MiB**, expected JEDEC **EF 40 17**, using the installed Teensy
**SerialFlash** library. `finalfsw/FlashHardware.h` isolates its ID, region and
enable switch. A mismatched ID disables storage. The entire chip is dedicated
to this journal; recognizable foreign contents stop writes. There is no erase,
format, circular overwrite or filesystem creation at boot or in the flight loop.

Project wiring is **CS10, MOSI11, MISO12, SCK13**, on default SPI, sharing data
and clock with SX1262 but retaining radio **NSS2**. Both chip selects are driven
high before initialization; drivers use SPI transactions. Verify the physical
wiring against these assignments: no schematic was present. Power the flash
at **3.3 V**, with common ground and inactive /WP and /HOLD as required by its
module. No 5 V connection to Teensy pins. Package-specific suffix/pin numbering
has not been established from the transcribed marking; use module signal labels.
The manufacturer specifies 64 Mbit, 2.7–3.6 V and ID 4017h:
[Winbond datasheet, pages 4 and 20](https://media.digikey.com/pdf/Data%20Sheets/Winbond%20PDFs/W25Q64JV_Rev_C.pdf).

RTC is **DS3231, RTClib, address 0x68, Wire SDA18/SCL19**. Checked reads reject
NACKs, oscillator-stop, invalid calendar, discontinuities and a stopped clock.
Lost power no longer silently sets compile time. GNSS UTC or GCS Set Time can
establish trust before launch; clock adjustment is rejected during an active
mission. An absent RTC does not stop sensors, telemetry, logging or monotonic time.

INA is **INA219, address 0x40, Wire1 SDA17/SCL16**. The supplied ESP32 reference
uses incompatible pins; they were not copied to Teensy. The small checked Wire
adapter in `AuxSensors.h` retains its 32 V / 2 A calibration (0.1-ohm shunt,
4096 calibration register, config 0x399F) and V/mV/mA/mW units. The optional
Adafruit INA library was not installed, so the adapter uses the same register
values and scaling from [Adafruit's implementation](https://github.com/adafruit/Adafruit_INA219/blob/master/Adafruit_INA219.cpp)
without adding a dependency. Each call performs one transaction, with failed
devices retried every 5 s; samples expire after 1 s. Missing INA gives `nan`
voltage on the existing telemetry field and UNKNOWN in the GCS, not a measured
zero. Current/power are provided through status; all four measurements are logged.
The shunt rating and actual I2C wiring remain bench checks.

## Mission time / recovery

Mission time is **unstarted at boot, connection, calibration and arming**. It
starts exactly at the existing **PAD -> POWERED_ASCENT** transition, with its
existing sustained acceleration/altitude or climb conditions unchanged.
`MissionClock` accumulates rollover-safe unsigned `millis()` differences into
a 64-bit elapsed count. It freezes when mission completion is recorded: LANDED
with no outstanding recording request, or after a transmitted manual/automatic
stop. Failed or ambiguous camera actions leave the mission incomplete.

On boot, the journal scans asynchronously before permitting arming. Each valid
record contains a complete mission snapshot. Active recovery restores the last
logged state, pad pressure, fused height/speed, peaks and camera request history;
it does not recalibrate an in-flight altitude to zero or reset the mission clock.
Elapsed time resumes from its last checkpoint plus DS3231 seconds only when the
checkpoint was UTC-trusted, the present RTC is valid and advancing, and the
forward gap is at most 24 hours. This has approximately one-second reset-time
quantization uncertainty, not millisecond accuracy. Otherwise elapsed time is
**UNCERTAIN**, not zero; flight processing continues in the recovered state.
Version-2 calibration records restore gyro and vertical-acceleration biases separately
from transient samples. Version-1 logs contain no such offsets; their in-flight ground-pressure reference remains recoverable. Long-reset motion cannot
be reconstructed from a last-known-state snapshot.

A reset never replays a camera command or resumes the automatic toggle timer.
The physical outcome between a committed intent and a UART/result checkpoint is
inherently ambiguous. Camera automatic actions are inhibited on recovery; inspect
the camera before any deliberate manual toggle. A recovered active mission cannot
be cleared by ARM. An armed, unlaunched mission requires the explicit confirmation
flow; a torn tail requires recovery review/confirmation; critical storage failure or foreign data blocks new arming.
Completed missions may be explicitly rearmed when stationary and recording inactive.
Full flash requires off-flight readout/maintenance; firmware does not erase it.

## Journal format and bounded service

Version 2 records are **128 bytes** (version 1 remains readable), little-endian, packed as defined in
`FlightJournal.h`: magic/version/type, sequence and mission IDs, boot time,
RTC/launch reference, 64-bit elapsed time and quality, state/flags, pad pressure,
height/speed/peaks, XYZ acceleration (m/s²), INA values, command sequence,
landing references, reset cause, drop count, intent detail, record length, telemetry reservation high-water mark, calibration generation, CRC32 and commit byte.
CRC32 is reflected polynomial EDB88320, init/final XOR FFFFFFFF, over bytes 0–122;
CRC occupies 123–126 and committed marker A5 occupies byte 127. Body programming,
read-back, commit programming and final validation occur on separate loop calls.

Recovery scans all slots, accepts committed CRC-valid versioned snapshots, skips
torn slots, and appends beyond the highest occupied slot. It never reuses a torn
slot or an erased hole before existing data. A damaged/foreign header causes a
conservative hold, preserving available earlier snapshots for recovery. Recovery
and reset outcomes are appended only when storage is writable; otherwise status
reports the failure. This is an inspected recovery path, not power-cut hardware
validation.

The queue holds eight records, routine sampling is **5 Hz**, and the last 32 slots
are reserved for critical events. Critical events can evict queued samples, never
an already-programming record. Launch, state, camera-intent/result, landing,
completion, reset/recovery and clock events are logged. A full chip stops writes;
existing records are retained. At 5 Hz the nominal 8 MiB sample capacity is about
3.6 hours before event overhead. Important events flush through the same bounded
service immediately; camera toggles wait for their intent to commit first.

No erase, ready-wait loop or delay was added to the flight loop. Flash service
performs a bounded status/read/page-program step; readiness times out after 100 ms.
I2C errors use the installed Teensy Wire driver's bounded transfer timeouts; a stuck
bus can still delay an individual transfer. This is not a hard real-time timing
validation.

The original 16 telemetry fields, timestamp semantics, command envelopes and ACK
framing remain intact. An additional roughly 5-second status downlink is:
`$SYS,team,mission,quality,elapsed_ms_or_NA,recovery,storage,rtc_valid,ina_valid,current_mA,power_mW,dropped,armed,auto_stop,telemetry_on,sequence_status*XOR`.
Quality is 0 unstarted / 1 monotonic / 2 RTC-recovered / 3 uncertain; recovery is
0 scanning / 1 new / 2 completed / 3 recovered / 4 hold; storage is 0 unavailable /
1 scanning / 2 ready / 3 fault / 4 full / 5 foreign. Automatic stop is 0 idle /
1 pending / 2 sent-unconfirmed / 3 failed-inhibited / 4 manual. ACKs retain priority.
A 250 ms receive window follows each downlink; an active incoming header is protected. These windows can defer telemetry slots.
The existing ground bridge forwards this without changes. The GCS interpolates
recent onboard time only, freezes stale data and never starts live time locally.

## Flash and use

1. In Arduino IDE with installed Teensy support, open **finalfsw/finalfsw.ino**,
   select **Teensy 4.1**, USB Serial, and verify/upload with all its headers.
   Required existing libraries remain RadioLib, BNO08x, BMP581, TinyGPSPlus,
   RTClib/BusIO/UnifiedSensor; SerialFlash comes with Teensy. Do not upload the
   ESP32 INA/RTC reference sketches to the flight board.
2. Keep the existing **inspacelora/lora_rx/lora_rx.ino** ground firmware with
   `CSV_ONLY=1`. No receiver change/reflash is needed if already installed. Its
   existing SPI/pin configuration assumes the original ESP32-style ground board.
3. Open **workswithoutcmd copy.html**, connect serial at 115200, keep the rocket
   still, and wait for flash **READY** and stable sensors. A full-chip scan can
   take several seconds. Missing/mismatched/foreign flash rejects arming.
4. Resolve legacy sequence migration if prompted (see REPAIR_NOTES). Use Calibrate and wait for DONE. Establish UTC using GNSS or **Set Time**. Click **Arm Mission** and accept the
   explicit new-mission confirmation. Wait for stationary pad initialization.
   **Reset State** remains a local UI reset and does not clear flight records.
5. With the camera already powered independently, click **Start Recording** once
   and observe the camera. Both recording buttons still send **CC 01 01 E7** on
   **Serial5, 115200 8N1, TX20/RX21**. This is the requested power-button toggle
   experiment, not a physically verified recording protocol. The first ACK may
   say CAM_PERSISTING; DONE confirms UART queuing only. No automatic retries.
6. The existing confirmed LANDED transition starts the onboard 60-second timer.
   At expiry it latches the attempt, commits a stop intent, then sends the same
   packet once. Timing is 60 s plus loop/commit/UART latency. A manual stop
   suppresses this automatic action. Disconnecting the GCS does not affect it.
7. Use Stop Recording for a deliberate manual test; inspect actual camera state.
   Button selection stays mutually exclusive across ACKs/timeouts/telemetry and
   local resets. Status remains requested/unconfirmed. After any MCU reset,
   inspect recovery status and the camera; automatic toggling is inhibited.

## Checks performed

`powershell -NoProfile -ExecutionPolicy Bypass -File tests/verify.ps1` checks actual
inline JavaScript syntax/handlers, command framing/ACKs/selection, onboard-time
display, CRC/packet assertions, estimator/timer and mission-clock compile-time
cases (boot, launch, rollover, trusted recovery, invalid RTC and completion). The actual journal is also simulated at compile time with an in-memory NOR backend for torn/corrupt records, packet reservations, calibration and migration.
The flight sketch compiles to an ARM object against the installed Teensy core,
with flash enabled and disabled. This does **not** link a flashable image, execute
the journal on hardware or verify a physical camera, sensor or power interruption.
Arduino IDE Verify/Upload and the bench checks above remain necessary.
