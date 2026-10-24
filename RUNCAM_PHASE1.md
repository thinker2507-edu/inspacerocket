# RunCam Phase 1 bench build

Historical Phase 1 notes. The current recording/fusion workflow, runtime camera
direct reference commands and landing timer are documented in [FLIGHT_RECORDING.md](FLIGHT_RECORDING.md).
The compile-time-only recording gate and GCS setting described below have been superseded.

Flash `finalfsw/finalfsw.ino` on the flight Teensy 4.1. It uses the inspected
`rocket_tx_teensy_cmd` sketch as its baseline; that original and the RTC-only
sketch remain untouched. Do not flash both sketches or combine their Arduino tabs.

## UART and packets

RunCam uses **Serial5 at 115200 baud**, default 8N1: Teensy TX pin **20** to
camera RX, camera TX to Teensy RX pin **21**, and common ground. The installed
Teensy 1.62.0 core's `HardwareSerial5.cpp` confirms these defaults. Camera power
and signal voltage must follow the actual camera's specification.

Existing SX1262, GPIO, SPI, I2C, GNSS/Serial7, RTC, flight states, telemetry
formatter and radio scheduler are preserved. No E32 UART implementation is used.

| Request | UART bytes | Availability |
| --- | --- | --- |
| Power button | `CC 01 01 E7` | Enabled; one click, no automatic repeat |
| Start recording | `CC 01 03 98` | Model verification required; disabled by default |
| Stop recording | `CC 01 04 CC` | Model verification required; disabled by default |
| Power off | None | No verified implementation; button disabled |

CRC-8/DVB-S2 is calculated over the first three bytes: polynomial `D5`, initial
zero, no reflection or final XOR. Independent check: ASCII `123456789` -> `BC`.
The existing CRC implementation was moved into `finalfsw/RunCamControl.h` and is
used by the single UART sender. Unsupported actions never reach Serial5.

## Evidence and unresolved camera behavior

The original firmware already used recording actions 03/04. RunCam's official
[Device Protocol](https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol)
documents them, with separate start/stop feature bits. Support on this camera
has not been established. Enable `RUNCAM_RECORDING_VERIFIED` in
`finalfsw/RunCamControl.h` and `CAMERA_CONFIG.RECORDING_VERIFIED` in the GCS only
after model/firmware documentation or device feature information establishes both.

The protocol defines action 01 as a power-button click, without a shutdown or
hold duration. Its press/release commands belong to the separate five-key OSD
interface. There is no evidence here for sending them as a power-off sequence.
The camera model, firmware version, physical-button behavior and relevant manual
are needed to determine shutdown. The reported failure has not been reproduced
or diagnosed on hardware. Do not repeat clicks or invent delays to force OFF.

## Command path

GCS Power ON -> `$CMD,23,4821,<seq>,CAMPWR*HH` + serial LF -> existing
`inspacelora/lora_rx` bridge -> unchanged command payload over SX1262 ->
`pollRadioRx` -> full-frame validation -> `handleCommand` -> `executeCommand`
-> `camPowerButton` -> `camSend` -> Serial5 -> `CC 01 01 E7`.

The flight reply uses the existing format
`$ACK,23,<seq>,OK,CAM_POWER_BUTTON_SENT*HH`. This means UART bytes were queued,
not that power changed. CAM,1 and CAM,0 use `CAM_REC_START_SENT` and
`CAM_REC_STOP_SENT`; unverified recording returns `REJECT,CAM_RECORDING_UNVERIFIED`.
Disabled/failed UART returns `FAIL,CAM_UART_FAILED_OR_DISABLED`.
Existing STATUS field `CAM` remains the last recording request flag, not feedback.

Example frames with sequence numbers for a deliberate test (LF on serial only):

```text
$CMD,23,4821,1,CAMPWR*6F
$CMD,23,4821,2,CAM,1*24
$CMD,23,4821,3,CAM,0*24
```

The GCS uses a random starting sequence and increments it per camera request.
It validates ACK framing/checksum/team/sequence, waits up to 5 seconds, and never
automatically retries. State remains UNKNOWN for all outcomes. Power ON is
latched after one attempt per page, including uncertain write/ACK failures.
Inspect the camera before reloading for another deliberate test. Existing flight
deduplication still covers only its most recent sequence; no broader replay
guarantee was added. The raw serial console remains a manual diagnostic facility.

## Ground bridge

Uplink is implemented in the existing receiver, preserving its configured SPI
18/19/23, NSS 5, DIO1 26, RESET 25, BUSY 27, SX1262 driver and radio settings.
Its actual board is still unconfirmed; its API/pins are ESP32-style. Confirm it
before selecting a build target. No board or additional GPIO was invented.

Default `CSV_ONLY=1` now means exact radio payload plus LF, for both telemetry
and ACKs. RSSI/SNR are no longer appended to machine payloads. Diagnostic mode
remains available but is unsuitable for GCS use. Uplink buffers at most 99 bytes,
rejects invalid framing/XOR/non-ASCII and overlong lines, discards timed-out
partial lines through LF, waits 50 ms, transmits once, and immediately rearms RX.
The delay is a bench starting point, not a guarantee against half-duplex clashes.
Flight validates complete payloads before command execution.

The browser accepts the command baseline's 16-field telemetry ending in state
text as rocket data, while retaining existing source-tagged layouts. No flight
telemetry bytes were changed. Noncamera GCS controls retain their previous legacy
commands; this phase does not resolve those previously identified mismatches.

## Verification and builds

Run from the project root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\verify.ps1
```

The runner supports `-NodeExe`, `-ArduinoData`, and `-ArduinoLibraries`. This
machine used VS Code's bundled Node 24.21.0 (`ELECTRON_RUN_AS_NODE=1`), installed
Teensy core 1.62.0, ARM GCC 15.2.1, RadioLib 7.8.1 and the existing sensor libraries.
The bundled runtime emits a Crashpad diagnostic but exits zero on passing tests.

Passed:

- C++ compile-time assertions on actual packet/validation helpers, with recording
  gated off and on: exact packets, standard CRC check, unsupported operations,
  strict uint32 values, partial frames, embedded NUL and invalid hex.
- Independent JavaScript polynomial-division CRC check.
- Actual inline GCS JS with mocked DOM, serial and timers: button mappings,
  recording guard, one power request, bad/wrong/late ACKs, ACK/reject/failure,
  timeout without retry, disconnect/simulation guards and serial-write failure.
- Telemetry compatibility fixtures for baseline rocket, source-tagged rocket and
  CanSat; exact preservation of 21 unrelated flight function bodies.
- Flight sketch compilation to an ARM object against installed Teensy libraries.

No linked firmware image, ground-board build, browser visual test, radio bench
test, camera test or flight test has been completed. Arduino CLI/PlatformIO are
not on PATH. Use Arduino IDE with Teensy 4.1, USB Serial, the installed libraries
and `finalfsw` for the complete Verify/Upload step. Select the ground target only
after confirming its board. Do not treat object compilation as a complete build.

## Manual bench procedure

1. Confirm the ground board/radio wiring and camera model/firmware. With the
   existing sensor hardware connected, compile/upload `finalfsw` and the ground
   receiver using `CSV_ONLY=1`. Confirm unchanged telemetry reaches the GCS.
2. Attach a logic analyzer to Serial5 TX20 and ground, decode 115200 8N1.
   Confirm the camera is in the known initial condition used in the working test.
3. Connect GCS serial at 115200. Click Power ON once. Verify one
   `CC 01 01 E7` frame, a matching flight ACK, and independently observe camera
   LEDs/display. An ACK alone does not establish power state. If it times out,
   inspect the camera before any new manual request; do not retry blindly.
4. Until recording support is established, confirm Start/Stop remain disabled
   and firmware rejects CAM commands without camera UART output. Once both
   capabilities are verified, enable both documented gates, rebuild, reload,
   establish the requested-on workflow, and click Start Recording. Capture
   `CC 01 03 98`, observe the recording indicator, then click Stop Recording,
   capture `CC 01 04 CC`, and verify the saved video independently.
5. Confirm Power OFF sends no bytes and explains the limitation. Use the
   camera's documented physical shutdown procedure. Record whether shutdown
   requires a hold, whether recording must first stop, its duration and the
   camera's model/firmware. Provide this evidence before adding remote shutdown.
6. Disconnect/disable the return link for an isolated test request. Verify timeout,
   UNKNOWN physical state and no retransmission. Restore the link and confirm
   telemetry still operates. Restore known camera state before another power test.
