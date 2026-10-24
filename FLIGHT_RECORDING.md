# Current flight recording-toggle test

RTC/INA integration, flash configuration, mission arming and reset recovery are
documented in [MISSION_STORAGE.md](MISSION_STORAGE.md). Current sensor/command/counter repairs are in [REPAIR_NOTES.md](REPAIR_NOTES.md). Those instructions apply
to the current firmware.

## Flash and use

1. Verify/upload `finalfsw/finalfsw.ino` for **Teensy 4.1**, retaining its
   companion headers and existing libraries. RunCam remains **Serial5, 115200, 8N1**:
   TX20 -> camera RX, camera TX -> RX21, common ground.
2. Keep the camera powered on independently. There is no GCS power workflow.
3. Use `inspacelora/lora_rx/lora_rx.ino` with `CSV_ONLY=1` on the existing ground
   board. This receiver is unchanged; no reflash is needed if already installed.
4. Open `workswithoutcmd copy.html`, connect serial at 115200, and leave the
   rocket stationary. Wait for flash READY, set UTC with GNSS or Set Time, click
   **Arm Mission** and wait for its stable-pad initialization.
5. Click **Start Recording** once. Its selected appearance persists. The GCS
   shows START REQUESTED, delivery progress and physical recording UNKNOWN.
   Observe the actual camera to determine whether the test toggled recording.
6. Click **Stop Recording** for a manual stop test. It becomes the sole selected
   button. After an ACK or timeout, both controls become available for another
   deliberate press. No automatic retry is sent.
7. For autonomous stopping, request Start and allow the existing flight state
   machine to reach confirmed LANDED. The Teensy sends one stop-intent packet
   at 60 seconds, without requiring the GCS or LoRa connection.

## Exact packets and limitations

The GCS camera section has only **Start Recording**, **Stop Recording**, and
**Camera Status**. Power ON/OFF, Camera / Flight Status, New Mission and the large
camera-debug section are removed. The camera is assumed already powered on.

| Intent | LoRa command body (existing team/key/sequence/checksum framing) | RunCam UART bytes |
| --- | --- | --- |
| Start | `CMD,23,4821,<seq>,CAM,1` | `CC 01 01 E7` |
| Manual Stop | `CMD,23,4821,<seq>,CAM,0` | `CC 01 01 E7` |
| Onboard automatic Stop | No ground request needed | `CC 01 01 E7` |

Both recording-operation aliases in `finalfsw/RunCamControl.h` are **0x01**.
This is the reference sketch's POWER BUTTON operation, used here experimentally
as a recording toggle. Operations 03/04 are not silently substituted. CRC remains
DVB-S2, polynomial D5, initial zero, over CC 01 01, producing E7. The existing
packet builder and UART sender are reused. No camera query or reply is required.

This test has not established whether 01 toggles recording, controls power, or
has another effect on the actual camera. The GCS never reports physically
confirmed RECORDING or STOPPED based on a flight ACK. It displays the user's
START REQUESTED / STOP REQUESTED intent, packet-delivery status, and UNKNOWN
physical state. The status describes requests, not a device-state measurement.

Selection lives in a separate `cameraSelection` variable. Only a Start/Stop
click changes it; rendering uses an active CSS class and `aria-pressed`.
ACKs, telemetry, timeout, disconnection, and local Reset State do not erase it.
Commands are temporarily blocked while disconnected, replaying, or awaiting an
ACK. Every subsequent deliberate permitted press sends one command with a fresh
sequence, even for the same intent. Existing most-recent-sequence deduplication
still prevents an immediate duplicate radio packet from toggling again.

A successful manual Stop suppresses the later automatic attempt; a failed manual
attempt also suppresses automatic repetition because the camera outcome can be
uncertain. Start after LANDED remains rejected until a new mission is initialized.
Telemetry, Calibrate and Set Time now send the existing firmware's TELEM, CAL,ALL
and TIME,UnixUTC commands with valid framing; UI success waits for the flight ACK.
Calibration waits for its terminal DONE/FAIL. Reset State clears local displays,
not flight state. Serial disconnect now releases the reader/pipe locks.

## Estimator and phases

The existing BMP581 runs at 50 Hz; fresh data-ready status is now polled before
reading each sample, with no new GPIO. The existing BNO085 accelerometer remains
at 100 Hz, gyro/rotation-vector at 50 Hz. A 50 Hz gravity report supplies local-up
direction in the same device coordinates as acceleration. Vertical acceleration is
`dot(acceleration - gravity, gravity / |gravity|)`, in m/s². No sensor axis or raw
acceleration magnitude is integrated as vertical motion.

Barometric reference initialization requires three seconds of fresh stable pressure,
at least 100 samples and less than 0.75 m pressure-height spread. It no longer
waits for IMU accuracy before making barometric altitude available. Reliable IMU
motion still rejects the reference. Flight arming and PAD entry require fresh,
stationary IMU and barometer data plus the explicit committed ARM workflow.
Saved version-2 calibration is recovered before a new reference can be created;
a reboot alone does not zero altitude. BOOT still uses PAD on the legacy wire
state field; SYS/STATUS distinguish unarmed/recovery/reference availability.
The lightweight complementary estimator predicts height/vertical speed with
gravity-removed acceleration. Barometric height and a filtered pressure-derived
speed correct drift (height time constant 0.35 s; speed correction 0.8 s;
barometric-speed filtering 0.25 s). Invalid/repeated/delayed IMU samples are not
integrated; pressure/temperature ranges, height jumps, integration gaps and
nonfinite values are guarded. Barometer age over 100 ms prevents new sensor-based
transitions; over 250 ms stops inertial extrapolation. IMU loss degrades to
barometric correction only; stale IMU prevents landing confirmation.

Existing telemetry field order and state numbers are unchanged. Altitude and
internal vertical speed now use the fused estimate.

| Transition | Condition |
| --- | --- |
| PAD -> POWERED_ASCENT | Five new barometer updates with fresh upward acceleration >1.5 g above gravity, fused height >3 m and speed >0; or five updates above 15 m and 10 m/s |
| POWERED_ASCENT -> COAST | Speed >3 m/s below its peak for five fresh barometer updates |
| ASCENT/COAST -> APOGEE | Height >1.5 m below its peak and speed <=0 for five fresh barometer updates |
| APOGEE -> PARACHUTE_DESCENT | Existing one-second apogee hold |
| DESCENT -> LANDED | Three seconds, at least 100 new pressure samples: fused and barometric heights within +/-5 m of pad, both speeds <1 m/s, pressure height spread <=0.5 m, fresh stationary IMU |

Stationarity requires 0.85–1.15 g, projected residual acceleration <0.5 m/s²,
gyro rates <5 deg/s per axis, valid accuracy and sample ages <=100 ms. LANDED is
terminal; noise cannot restart ascent or the timer. An explicit `ARM` command from the serial console initializes another mission, permitted only on PAD/LANDED with fresh stationary sensors,
no calibration result pending, and no active recording request. It clears
flight detection, estimator, landing timer and recording diagnostics, then
reacquires the stable pad reference. It never sends another power click.
Power cycling now recovers persistent mission state and inhibits automatic camera
toggles; it does not initialize a new mission. Barometer recalibration reacquires
the estimator reference without resetting a running flight (CAL requires prelaunch stationary BOOT/PAD).

## Automatic stop and diagnostics

On the single transition to LANDED, landing `millis()` is latched. On the first
loop pass with unsigned elapsed time >= **60000 ms**, the attempt latch is set
before issuing CC 01 01 E7, provided Start was successfully queued.
There is no blocking wait and no automatic
retry after failure. Actual timing is 60 seconds plus loop/UART scheduling latency,
not a hard real-time guarantee. Flash intent commitment adds a short scheduling
latency. Timing survives millis rollover; a reset inhibits automatic camera toggles
while recovering the mission reference and last logged flight state.
There is no automatic recovery out of LANDED; an allowed explicit new mission
cancels the old timer.

The existing STATUS command gains optional argument CAMERA. Its ACK information
uses semicolons; default STATUS and telemetry remain compatible:

| Field | Meaning |
| --- | --- |
| REF | 1: reference operation-01 toggle test is enabled; not device-state confirmation |
| SR / ST / SF | Start requested / queued to UART / flight request ACK queued |
| TT / TF | Stop queued to UART / manual stop flight ACK queued (automatic stop has no request ACK) |
| L | Confirmed landing milliseconds since this boot; zero before landing |
| AS | 0 idle; 1 countdown pending; 2 stop sent, unconfirmed; 3 stop failed, no retry; 4 manual stop attempted |
| CA | NA: no recording-control camera ACK defined |
| E / R | Estimator 0 unavailable, 1 barometer only, 2 fused / estimate currently ready and fresh |

These optional STATUS,CAMERA diagnostics remain available through the serial
console. The compact camera display shows request/delivery status only.

## Essential checks and limitations

Run `powershell -NoProfile -ExecutionPolicy Bypass -File tests\verify.ps1`.
The focused checks cover packet E7 for both intents, persistent/exclusive button
selection, framed noncamera controls, event handlers, stream disconnect cleanup,
and ARM translation-unit compilation. No large test campaign or physical camera
test was run. No linked image or ground build was generated;
complete Verify/Upload in Arduino IDE.

These are untuned flight thresholds, not hardware-validated flight estimates.
The BNO085's gravity estimate can be inaccurate during sustained high acceleration,
vibration or saturation. Check stationary projection and sensor freshness on the
actual assembly. Pressure port airflow and temperature matter. Landing more than
5 m above/below launch reference, continued movement, sensor loss or unreliable
IMU accuracy can prevent landing confirmation and therefore prevent automatic
stop. A reset recovers logged recording-request history and mission state but
inhibits automatic camera toggles because the physical outcome is unverified.

Protocol evidence: [RunCam Device Protocol](https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol),
[Betaflight protocol definitions](https://github.com/betaflight/betaflight/blob/master/src/main/io/rcdevice.h).
Sensor evidence: [CEVA BNO08x datasheet, section 2.1.1](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf),
[Bosch BMP581 datasheet, interrupt sources](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf).
