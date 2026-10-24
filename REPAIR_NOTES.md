# Telemetry, calibration, commands and reset recovery repairs

## PWM4 continuous rotation; PWM5 independent control (2026-10-22)

User confirms PWM4 has a continuous-rotation MG995. Added PWM4 SPIN (configured
1700us), STOP (nominal neutral 1500us) and REVERSE (1300us), through the same
authenticated GCS/LoRa path and two-stage ACK/DONE. PWM5 keeps its existing
independent numeric/SAFE/TEST90 controls. No new GCS controls or guessed full-turn
timer. Continuous type is reported by STATUS SERVO C4/C5. PWM4 TEST90 remains a
legacy nominal-neutral command, not a requested angle. Speed, stop trim, physical
direction and exact turns are unverified. Automatic deployment flags are unchanged.
See commands.md (short list) and COMMAND_PROTOCOL.md (details).

## Wireless PWM ACK ordering (2026-10-22)

Rocket radio PWM commands now validate and queue before their initial acceptance
ACK, wait for matching ACK TX completion, then issue PWM and return DONE over
LoRa. A failed acceptance TX starts no output. Direct USB remains immediate.
The existing GCS command input sends PWM commands normally; its ACK handler waits
for DONE after acceptance without adding controls or changing layout/interlocks.
STATUS LINK identifies this firmware with SERVO=ACK_FIRST_1. Install the updated
rocket firmware and reload the GCS; no servo-specific receiver update is needed.
No direct rocket USB Serial Monitor is needed for operation. Real-world timeout
cause and physical motion/power behavior remain unverified. See COMMAND_PROTOCOL.md.

## GCS servo layout reverted (2026-10-22)

Removed all added servo buttons, panel, handlers and status hooks from the main
rocket GCS, restoring its previous layout and script behavior. Servo firmware
commands remain available through the existing console. Optional servo-commands.js
is a DevTools snippet using the existing encoder/transport/ACK path without adding
UI. Main-GCS layout changes need explicit user agreement. See commands.md.

## Rocket servo controls and maintained handoff (2026-10-22)

Added independent PWM4 (Teensy22) / PWM5 (Teensy23) native PWM control in
`finalfsw/ServoControl.h`, normal authenticated USB/LoRa commands PWM4/PWM5
(TEST90, SAFE or bounded microseconds), STATUS,SERVO and rocket GCS controls.
User identified MG995; TEST90 uses nominal 1500us center at 50Hz, with bounded
1000..2000us manual pulses. This is not a measured relative 90-degree rotation.
Startup signals stay LOW; automatic/startup mechanical positions are unconfirmed.
Per-output startupSafeConfirmed/deploymentConfirmed remain false pending that
confirmation. No new test mode or changes to existing rejection rules were added.

The existing APOGEE->DESCENT transition reserves a single servo intent in the
existing STATE checkpoint, commits before PWM and inhibits replay on reset or
ambiguous recovery. Automatic output is independently configurable, and tested
with simulated confirmed profiles. Manual tests never set/reset the auto latch.
No telemetry/CSV/journal layout, radio settings or CanSat implementation changed.
See `commands.md` for every current command and exact wire forms; `feature.md`
for architecture, hardware assumptions and handoff. Update both with relevant
future changes. Earlier chronological notes below can be superseded by them.

Verification: complete updated tests/verify.ps1 passed, including servo controller
and actual NOR recovery assertions, actual GCS frame/ACK/guard tests, both Teensy
ARM objects with flash enabled/disabled and receiver ESP32 syntax/API checks.
Fake CanSat's separate check passed. No linked image/upload or physical servo,
mechanism, flight-detection qualification or deployment test is claimed.


## Teensy direct-USB / GCS command path repair — USB_CMD_2

Verified software failures were at the Teensy USB boundary, not in the untested
ESP32 simulator: the flight USB console understood only a few text commands and
ignored GCS `$CMD` frames; its 64-byte line buffer could not accommodate the
99-byte wire-command limit; and USB had no independent machine-readable ACK route
for immediate or delayed command results. A failed radio initialization also
halted setup indefinitely. The GCS Reset State button only cleared UI data.

Both Teensy targets now use local `ConsoleCommand.h` for bounded 99-byte USB
lines, LF/CR/CRLF, invalid/partial-line rejection, comma/space forms, case
normalization and TELEM ON/OFF aliases. Local text is converted to the existing
authenticated command envelope and passed to the same `handleCommand` used by
LoRa. Received raw frames are validated as-is; a bad key/checksum/address is never
rewritten to make it valid. USB local command IDs are separate from telemetry IDs.

USB replies are raw `$ACK` packets and do not overwrite or wait for a radio ACK.
Calibration, recording and PAD-reset operations remember the request's connection,
so their final DONE/FAIL follows that connection even while radio TX is busy.
SYS packets are mirrored to USB. No radio frequency, pins, schema, authentication,
mission/calibration/counter recovery rules, camera operation bytes or CSV logging
format have changed. Radio init failure now leaves sensors, logging and USB
commands active; STATUS,LINK reports RADIO=0 and USB telemetry uses the same
formatter/reservation gate. No commands are automatically retried.

Both GCS consoles now accept `TELEM OFF` as well as `TELEM,0`, `CAL BARO` as well
as `CAL,BARO`, and lowercase input. COUNT aliases STATUS,COUNTER; RESETCOUNT remains
USB-only. The Reset to PAD button issues RESETPAD,CONFIRM and waits for durable
completion instead of merely changing labels/clearing plots. Simulation reset
retains its separate local behavior. Camera controls/selections remain unchanged.

### Verify the actual board

1. Flash `finalfsw/finalfsw.ino` (Teensy 4.1) with all its local headers. Use the
   CanSat sketch only for a CanSat flight Teensy. No ESP32 change is needed for
   this direct-USB repair.
2. Direct flight-board USB Serial Monitor: 115200, Newline or Both NL & CR.
   Send `HELP`, then `STATUS LINK`. The reply must include BUILD=USB_CMD_2.
   `STATUS`, `STATUS IMU`, `STATUS SD`, `CAL BARO`, `TELEM OFF`, `TELEM ON`,
   `RESETPAD CONFIRM` and `REBOOT` now all reach the common handler. Safety/storage
   rejections are returned explicitly; an OK acceptance is not physical execution.
3. Close Serial Monitor (and any other app using that COM port), open the updated
   rocket GCS (`workswithoutcmd copy.html`) and connect to that same Teensy USB.
   Only one application can own the serial port. On-connect STATUS must ACK;
   then telemetry buttons can reflect the firmware's reported state.
4. In GCS send `STATUS,LINK`; then test OFF and ON on an unarmed stationary PAD.
   Routine telemetry stops/resumes; SYS and command replies continue. Send one
   operation at a time and wait for DONE where calibration/PAD reset requires it.
   RESETPAD still blocks active-flight resets; REBOOT requires unarmed PAD.
5. If using the existing ESP32 receiver instead, select its COM port and retain
   its repaired uplink firmware. Direct-USB tests do not verify physical RF.

Checks: tests/verify.ps1 passed, including new constexpr tests of the actual USB
line collector/normalizer, GCS-to-Teensy dispatcher/reply-function simulation,
OFF/ON scheduling with/without radio, existing authentication/identity/replay,
mission/calibration/counter recovery and GCS tests. Both actual sketches compiled
to ARM objects against installed libraries (flash enabled/disabled), without new
warnings; receiver syntax/API check passed. The dispatcher simulation substitutes
sensor actions and C string tokenization; C++ framing helpers are separately
checked. No linked image, upload or physical Teensy/radio/sensor test is claimed.


## Explicit unarmed PAD reset

Added RESETPAD,CONFIRM through the existing authenticated GCS path and RESETPAD
CONFIRM on flight USB. Both real targets commit a dedicated mission-reset record
before reporting DONE; it survives reset and leaves PAD unarmed, mission time
unstarted, phase/landing/camera timers cleared and ground reference reacquiring.
It retains calibration records, telemetry settings, packet numbers and log history.
In-flight/ambiguous active states, camera/calibration operations and unhealthy or
busy storage are rejected with reasons. GCS waits for durable completion and
updates PAD/NOT LAUNCHED. The fake CanSat can hold synthetic PAD until USB FLIGHT.
See MISSION_STORAGE.md. Checks: both ARM sketch objects, ESP32 syntax/API compile,
GCS ACK/state tests, actual guard simulations and journal torn/committed reset
recovery assertions; no physical hardware testing.


## CSV SD logs and software reboot paths

Both Teensy SD loggers now create exclusive R/Cxxxxxxx.CSV files with named
columns/units, readable events, explicit invalid values and per-row CRC32. Internal
flash recovery layout is unchanged; old BIN files are preserved. Calibration
biases are exported separately from measurements. See [SD_LOGGING.md](SD_LOGGING.md).

The direct Teensy USB console had no REBOOT handler; the new fake CanSat explicitly
rejected REBOOT as unsupported. Both paths now implement it. Teensy USB reuses the
same authenticated-radio handler's safety policy (completed recovery, unarmed PAD,
no launched mission/calibration/camera action pending). ACKs precede restart;
routine TX/new samples pause while SD/NOR queues drain, with a 2-second storage
fallback and Cortex-M7 reset barriers. Fake CanSat supports USB/GCS REBOOT through
esp_restart and retains its NVS packet reservation. Physical reboot/card behavior
has not been tested; builds and focused simulations only.


## Explicit USB packet reset and OFF/ON counter behavior

`RESETCOUNT CONFIRM` now commits a new numbering epoch over local USB before
resetting count to zero; `TELEM OFF`, `TELEM ON` and `COUNT` are also available in
Serial Monitor. Reset is guarded against active missions/arming and pending work.
OFF freezes routine numbering and range allocation; ON continues rather than
resetting. The updated GCS pages distinguish a deliberate epoch reset from delayed
packets, retaining plots/history and resetting RX statistics for the new epoch.
See [PACKET_COUNTER.md](PACKET_COUNTER.md) for exact steps and compatibility.
The fast startup flash index remains unimplemented; startup recovery still scans.


## Flight logs moved to onboard microSD

Both targets write their 5 Hz flight sample log to the Teensy 4.1 onboard SDIO
microSD slot. External W25Q64 flash retains recovery checkpoints only (including
5-second in-flight state checkpoints); routine samples never fall back to flash.
Insert a formatted card before boot; `STATUS,SD` reports state, filename, write/drop
counts and slow calls. Existing flight/calibration/counter recovery is preserved.
See [SD_LOGGING.md](SD_LOGGING.md) for format, failure behavior and timing limits.
Flash fast-start indexing and the requested USB counter-reset command remain
unimplemented; this change does not erase existing logs or reset counters.


## RTC-final-style IMU telemetry

Per the requested reading behavior, both targets now copy each decoded acceleration
and gyro report directly into the telemetry cache before considering timestamp,
accuracy, range, stationary state or flight state. Acceleration is m/s^2 as in
`rocket_tx_teensy_rtc_final`; gyro is converted from rad/s to deg/s using the same
57.29578 factor. Existing explicit CAL,GYRO offsets remain applied to the emitted
gyro values, preserving the earlier calibration repair. Values are held between
reports, like the RTC reference; they are not replaced by `nan` when an IMU-state
or age check fails. Before the first report, the cache is `nan` rather than a
fabricated measurement. A sensor that stops reporting leaves the last received
values on telemetry. The trusted flight/fusion/calibration path remains separate
and still rejects stale or unreliable samples. No GCS IMU-state query or special
mode is needed to display the axes. The corrected six flight markers and source
IDs remain unchanged. Focused tests/compilation cover both targets.

## IMU NaN regression and GCS state markers

The old `rocket_tx_teensy_rtc_final` reads decoded BNO085 axes without comparing
their SH-2 timestamps to Teensy uptime. The newer firmware added that check, but
the installed Adafruit I2C HAL's `i2chal_read()` never assigns its `t_us` output.
`shtp_service()` initializes that value to zero, so `touSTimestamp()` is not based
on host uptime. Assuming otherwise incorrectly rejected real sensor reports.
This explains a concrete software path to persistent NaNs; physical behavior has
not been retested. The newer quality gate also hid fresh numerical readings when
calibration accuracy was zero. Those readings and calibration reliability are
now handled separately.

The installed `getSensorEvent()` uses one output object for every callback during
`sh2_service()`, returning only the last decoded report in a bundled packet. That
can discard other sensor types, including acceleration, gyro and gravity.
Local `FlightImu.h` adapters reuse the existing Adafruit I2C transport, reset
handling, SH-2 engine and decoder, supplying the missing receive timestamp and
queuing every decoded requested report. No installed library, address, sensor
model, bus, pin or sample rate was replaced. Each target has its own adapter.

`ImuSample.h` validates flight-only finite/range-checked data and sensor sample age, ignores
duplicates without extending their lifetime, and handles clock rollover. Fresh
numerical acceleration/gyro are transmitted even at accuracy 0 (UNRELIABLE);
fusion, stationary detection, arming and CAL retain the accuracy >=1 requirement.
Acceleration remains m/s^2 including gravity in device coordinates; gyro remains
deg/s with the existing stored biases applied. Direct telemetry now holds the
latest reports as described above; flight consumers retain stale-data rejection. Report reset
recovery clears old data and reinstalls subscriptions one per service invocation.
If required data stops, report requests are reissued at a bounded two-second
interval; failed configuration writes are counted. No blocking reset/begin/delay
or Wire2 bus reinitialization was added to the flight loop. An IMU absent at boot
remains explicitly unavailable rather than being faked as healthy.

Use the GCS command console **STATUS,IMU** for a checked ACK containing:

- BNO: initialization found the sensor; CFG=F: all four report requests were sent
  successfully (not a physical measurement confirmation).
- A/G/V: fresh numeric acceleration, gyro and gravity flags; Q: their calibration
  accuracy 0 unreliable / 1 low / 2 medium / 3 high.
- AGE: sample ages in ms, capped at 9999; DROP: report queue overflow; DEC: decode
  failures; RST: reset notifications; ERR: failed report-configuration writes.

The corrected GCS mapping in both pages is 0 PAD, 1 POWERED_ASCENT, 2 COAST,
3 APOGEE, 4 PARACHUTE_DESCENT, 5 LANDED. COAST no longer appears as ASCENT, and the
unused probe-release marker is replaced by COAST in the correct order. Default
STATUS's leading ST field restores the marker even when telemetry is OFF. Camera
diagnostics' differently scoped ST flag does not alter flight state. Old replay
CSV names LAUNCH_PAD/ASCENT/DESCENT remain accepted aliases; invalid live numeric
states are rejected. Marker state does not change camera button selection.

Rocket telemetry field 15 now carries source **1**, matching CanSat source **2**.
Field 14 remains the numeric state, and every other field, address and checksum
rule is unchanged. Rocket packets end `...,0,1*XX` on PAD instead of `...,0,PAD*XX`;
XX denotes the calculated XOR. No `PAD*` text is transmitted as the sensor packet
suffix. Historical addressed/checksummed state-text packets are still parsed for
compatibility, but the new flight software emits source IDs. Flash formats,
mission clocks, durable numbering, camera operations and radio settings remain.

Focused checks cover actual queue/sample helpers, duplicate/stale/nonfinite data,
accuracy separation, rollovers, each of the six GCS markers, source IDs, replay
aliases, STATUS synchronization and camera-status separation. Both ARM translation
units compile with flash enabled/disabled; existing GCS/isolation/recovery and
ground syntax checks pass. These are software checks, not physical IMU tests.
Flash the applicable updated sketch and open the updated corresponding GCS. Keep
the unit still, check finite axes and STATUS,IMU, then gently rotate it and check
gyro response. At rest acceleration magnitude should be near 9.81 m/s^2, not all
zero. Persistent missing data still requires checking actual BNO power/I2C wiring
and the reported diagnostics; no claim of hardware repair is made.

## Faster flash recovery

The original full-chip scan read one 128-byte slot per flight-loop pass: 65,536
reads/passes for 8 MiB, plus a final completion pass. Time spent revisiting radio,
GNSS and I2C service between each tiny read made recovery unnecessarily slow.
Both journals now read/check **4 KiB (32 records) per service call**, reducing this
to **2,048 bounded scan reads/passes**. The loop returns between chunks so sensors,
commands and other work continue. Normal page programming/verification is unchanged.
The extra recovery buffer costs 4 KiB RAM per flight computer; no new library,
SPI speed change, flash erase or record-format change was introduced.

The whole region is still checked, including valid data beyond erased gaps. CRC,
commit, legacy-calibration/mission recovery, maximum sequence reservation and
damaged-slot range burning are unchanged. No early blank-tail guess or unsafe
packet-number reset is used. Fault/full/foreign storage still reports its actual
condition; this change does not bypass recovery or durable arming safeguards.

Teensy USB prints one completion diagnostic:
`# FLASH scan: <elapsed ms>; <bytes checked> bytes checked; status=<number>`.
Healthy full-chip verification reports 8388608 bytes and status 2 (READY), or 4
if the journal is full. Status 3 is FAULT and 5 is FOREIGN. Time is measured from
the first scan service, including intervening flight-loop work, not sensor setup
before scanning or any separate wait to validate advancing RTC time afterward.
Actual hardware scan time is not yet measured; 32 times fewer passes/transactions
does not promise 32 times lower wall-clock time. Sensor stabilization and RTC
validity checks remain independent startup requirements.

Focused `tests/persistence.cpp` checks run against both actual journal headers:
one 4 KiB read per scan call, record recovery across erased gaps and the 4 KiB
boundary, torn/corrupted records, retained calibration/mission and conservative
sequence recovery. Existing OFF/ON, isolation, recovery and compile checks passed.

## PAD-only startup

Both targets now initialize directly in unarmed ST_PAD; the separate internal
ST_BOOT state has been removed. ARM resets pad initialization in ST_PAD. Launch
detection requires recovery complete, no recovery hold, explicit arming and a
committed ARM record, as well as the existing reference/sensor conditions. A
reboot still restores an interrupted mission instead of treating it as a new one.
Older journal records remain readable; state numbers 0..5 and telemetry layout
are unchanged. TELEM,1 is accepted regardless of flight/arming/recovery state;
only OFF has the existing recovery and in-flight restrictions. Removing BOOT
does not bypass address/key/checksum validation or add a USB command transport.
USB serial currently mirrors routine packets after successful LoRa TX; command
input remains the existing LoRa/ground-receiver path. Earlier BOOT/PAD descriptions
below are historical. Focused checks assert PAD startup, the durable arming gate
and ON acceptance during startup/recovery, with both flight compile checks.

## Rocket / CanSat coexistence follow-up

See **INTERFERENCE_NOTES.md** for the current addressed protocol and radio risk
analysis. Both flight targets and GCS pages now require 23:R / 23:C addresses;
legacy unaddressed live frames are deliberately rejected. Telemetry retains 16
fields with the qualified address in field 0 and a new CSV-body XOR suffix.
Command authentication/checksums, safety restrictions, packet reservations, flash
formats, calibration and launch-only mission recovery remain in place. The ground
receiver now tracks uplink receive windows by destination. Both radios retain
their original RF/pin configuration and add bounded scheduled-traffic jitter and
peer quiet windows. Earlier notes below describe the prior repair stages; upgrade
both flight targets, receiver and GCS together for this protocol change.

## Telemetry transmission follow-up and independent CanSat target

The periodic scheduler already checked `telemetryOn`, but `sendTelemetry()` lacked
a final guard and periodic SYS still carried INA current/power while OFF. The
packet builder now returns before consuming a number or transmitting when OFF.
`TelemetryControl.h` supplies the scheduler's rollover-safe due check. Both use
the same `telemetryOn` variable; there is no second display-only enable state in
firmware. Periodic SYS retains health/mission/recovery/actual-TX status, with INA
measurement values `nan` while OFF. Explicit requested status/ACK responses remain
available. No sensor acquisition, flash or flight-loop work is gated by this flag.

Path: GCS `toggleTelemetry()` -> `sendFlightCommand(TELEM,0|1)` -> existing XOR/
team/key frame -> ground `serviceUplink()` -> `pollRadioRx()` / `handleCommand()` /
`executeCommand()` -> authoritative `telemetryOn` -> `handleLoRa()` -> guarded
`sendTelemetry()`. The only radio transmit callers are numbered telemetry, ACK,
and essential SYS. ACK handling has priority and bypasses the routine-data gate;
OFF cannot suppress its own acknowledgement. The existing receiver needs no
change: raw forwarding and its bounded idle uplink fallback already support ON
while routine telemetry is absent. SX1262 settings and packet reservations remain
unchanged. ON builds a current packet rather than draining an outbound queue.

The rocket GCS now shows requested and last-confirmed telemetry state separately.
Lost ACKs/bridge errors/disconnect mark the current state UNKNOWN without erasing
the last-confirmed value; STATUS on connect and subsequent SYS resynchronize it.
Clicks and rerenders never optimistically change confirmed transmission state.
Repeated OFF/ON are assignments, not toggles. Existing safety/boot policy remains:
boot ON, launch forces ON, OFF in flight rejected, no counter reset on boot/toggle.
OFF also waits for journal recovery (RECOVERY_SCANNING rejection), so it cannot
be accepted in apparent BOOT and then reversed when an in-flight mission is found.

`cansatfswfinal/` is a separate, camera-free target with local baseline headers,
its own GCS, existing CanSat source tag 2 and a distinct CFL1 flash signature.
See its README for build/use, storage separation and missing PSMS hardware details.
PSMS is explicitly UNCONFIGURED, not a simulated sensor. The rocket implementation
was not altered for CanSat-specific behavior.

Focused checks: `tests/telemetry-check.cjs` extracts actual command/scheduler/gate
control flow, substitutes hardware calls and simulates 30 seconds OFF per target:
zero routine transmissions or sequence consumption, ACK/health retained, ON
resumes, repeated assignments and in-flight rejection preserved. This is not RF
simulation. `tests/targets-check.cjs` checks both actual GCS scripts, requested/
confirmed state, timeout/SYS recovery, handlers and CanSat camera-command blocking.
`tests/persistence.cpp` is compiled against each target's actual journal separately
for reset, torn/corrupt records, calibration, mission recovery and counter safety.
`tests/verify.ps1` compiles both flight translation units with flash enabled and
disabled, plus the existing ground syntax/API check. No hardware testing or
fully linked/uploadable-image build is claimed.

## Verified causes and changes

- BMP581: the installed Bosch `bmp5_set_osr_odr_press_config()` enters standby
  for ODRs above 5 Hz. `setupBMP()` changed OSR/ODR without restoring NORMAL,
  preventing DRDY/pressure updates. Configuration results are now checked and
  NORMAL is explicitly restored and verified last. Sensor/driver/pins remain
  SparkFun BMP581, Wire2 SDA25/SCL24, addresses 0x47 then 0x46, 50 Hz, Pa/°C.
- Altitude was zero until IMU-dependent pad initialization. `updatePadReference()`
  now establishes a stable pressure reference independently of IMU availability;
  IMU motion still prevents reference acquisition when reliably detected. Flight
  arming still requires fresh stationary IMU and barometer data. Recovery restores
  the saved reference before allowing initialization. No automatic pressure drift
  overwrites an explicit reference. The complementary estimator remains in use.
- `CAL,BARO` discarded its freshly applied pressure reference by recreating the
  estimator and starting another reference acquisition. `CAL,ALL` could partially
  apply pressure before checking gyro data, and counted repeated gyro samples.
  Calibration now collects unique fresh samples, validates every requested set,
  commits a dedicated record, applies the complete candidate, then sends DONE.
- The GCS assumed telemetry OFF; firmware defaulted ON. BOOT was reported as PAD
  while CAL/OFF rejected that internal state. The GCS now synchronizes through
  STATUS and SYS; safe prelaunch BOOT calibration/OFF are allowed with the same
  sensor/motion/mission checks. OFF during flight remains rejected. No optimistic
  toggle, retry or telemetry-arrival-based reversal is used.
- Ground uplink timing depended on click time rather than observed receive
  windows; flight downlinks could truncate an incoming command. The bridge waits
  for a downlink window (bounded idle fallback supports TELEM,1 while OFF), sends
  once asynchronously, and forwards raw packets. Flight reserves a 250 ms receive
  window and protects an incoming header with a bounded timeout. Radio settings
  and pins are unchanged. Both endpoints use `CommandValidation.h`.
- Console input bypassed the command envelope/ACK tracker; it now supports
  NAME[,arg] and valid complete $CMD frames through the same command path.
  Ground bridge errors and specific flight rejections are visible. CSV recording
  previously replaced transmitted packet numbers with receive counts; it now
  retains the packet number from field 2 (zero-based).
- Packet count existed only in RAM. Durable reservations now prevent reuse after
  reboot; calibration records are independently retained alongside flight state.

## Altitude and calibration semantics

Pressure -> `relativePressureAltitude(p,p0)` -> complementary estimate -> telemetry
field **3** (zero-based, fourth CSV field) -> unchanged receiver payload ->
`parsePacket()` -> `ingestRocket()` / altitude plot. Height is metres relative to
the calibrated ground pressure, not GNSS altitude or absolute sea-level height.
Invalid/uninitialized/stale altitude, pressure or temperature measurements
are transmitted as `nan`; IMU axes follow the latest-reading policy above. The
GCS retains NaN/gaps and displays INVALID/STALE for altitude.
Loss of packets produces a stale indication; acknowledged OFF shows PAUSED.

CAL is accepted only in prelaunch BOOT/PAD with fresh, reliable stationary IMU
measurements. Three seconds and at least 100 distinct samples per requested
quantity are required; barometer spread must stay within 0.75 m. BARO averages
reference pressure; GYRO averages/subtracts three biases in deg/s; ACCEL averages
the gravity-projected vertical residual in m/s². ALL performs all three. Raw
accelerometer axes still contain gravity in device coordinates and are not
blindly zeroed. INA/RTC/GNSS are not zeroed. The filter and launch/stationary
logic apply the vertical bias. Gyro-only calibration leaves altitude filtering
and the pressure reference intact. Missing/stale/moving data leave old parameters.

Calibration payloads use a distinct LOG_CALIBRATION record with generation,
CRC/commit and validated pressure/gyro/vertical-bias fields; later sample records
cannot replace those offsets. DONE means applied after commitment. If flash is
unavailable, a bench calibration can apply in RAM but explicitly reports
APPLIED_VOLATILE_STORAGE_UNAVAILABLE; it does not enable durable flight arming.

## Packet counter and reset rules

Version **2** retains the **128-byte** record size, CRC32 over bytes 0–122 and A5
commit byte at 127. It adds length, telemetry reservation high-water mark and
calibration generation within the old reserved space. Version-1 records remain
readable. Journal sequence, command sequence and telemetry packet sequence are
separate uint32 values. No startup erase or circular overwrite was added.

Before sending any numbered telemetry, reserve **64** numbers durably. First
fresh sequence is 1. Reboot skips the unused tail of the last reservation:
after packets 1–7, the next boot starts at 65. Gaps are intentional; perfect
continuity is not claimed. All valid records contribute to the recovered maximum.
Each damaged slot burns another 64 numbers to cover a concealed reservation.
No number is sent from an uncommitted range. UINT32_MAX exhausts transmission
instead of wrapping. Missing/full/foreign storage pauses numbered telemetry when
no safe range remains; unnumbered ACK/SYS diagnostics and flight processing continue.

**Version-1 migration:** old logs never stored packet numbers. If the GCS says
LEGACY BASELINE REQUIRED, review the previous transmission history and enter
`COUNTER,<known-safe-upper-bound>` in the console. This must cover every previously
used number, including potentially unreceived packets, not merely the last
received CSV row. The baseline is authenticated and committed before new ranges
are used. It cannot decrease/reset an already recovered counter. If a reliable
upper bound is unavailable, no safe historical continuity can be claimed; preserve
the logs and resolve that baseline before flight. Unknown/corrupt history is
retained as such across further resets. ARM rejects an unresolved counter.

Mission time still begins only on PAD -> POWERED_ASCENT. Recovery restores the
last valid flight state/reference and trusted RTC elapsed time, otherwise marks
time UNCERTAIN. A reset is not a new mission. Camera operations are never replayed
on recovery. The normal onboard 60-second landing stop remains one-shot, uses
CC 01 01 E7 on Serial5, and is independent of the GCS. Physical camera state is
still unverified; an MCU reset inhibits automatic toggles.

## Use and focused verification

Upload **finalfsw/finalfsw.ino** to Teensy 4.1 and the updated
**inspacelora/lora_rx/lora_rx.ino** to the existing ground board. Keep this project
tree together so the receiver can include the shared validation header. Open the
existing GCS, connect, wait for recovery, resolve a legacy baseline if prompted,
and keep the rocket still. Calibrate -> wait for DONE -> Set Time/GNSS UTC ->
Arm Mission -> wait for pad initialization -> Start Recording. Reset State only
clears local displays. TELEM OFF/ON controls flight telemetry transmission, never
sensor acquisition, logging, command reception or the local landing timer.

Run `powershell -NoProfile -ExecutionPolicy Bypass -File tests/verify.ps1`.
It checks actual inline JS/handlers, command framing/status, altitude/NaN/CSV
mapping, packet CRCs, estimator/timer logic, and actual journal code with a
constant-evaluated NOR backend (reboot, torn/corrupt records, calibration recovery,
legacy migration, counter exhaustion). It also compiles the flight sketch to an
ARM object, enabled/disabled flash variants, and checks the ground sketch against
installed ESP32/RadioLib headers. These are software checks, not linked images or
physical tests. No sensor/radio/flash/camera/power-cut hardware test was performed.
