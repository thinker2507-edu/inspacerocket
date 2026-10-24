# Implemented commands

Maintained targets: rocket `finalfsw/finalfsw.ino` (R), CanSat `cansatfswfinal/cansatfswfinal.ino` (C), and synthetic CanSat `datafakecansat/datafakecansat.ino` (F). F substitutes for C; never operate both identities together. Historical sketches are covered separately below. This file describes source behavior, not hardware validation.

**Maintenance rule:** update this file in the same change whenever a command is added, removed, renamed, or changes format, behavior, restrictions, or effects. Update `feature.md` for related architecture/hardware changes. Do not list proposed commands as implemented.

## Shared envelope, transport and results

For R/C/F, the exact command envelope is:

```text
$CMD,23:R,4821,<seq>,<NAME>[,<arg>]*HH
$CMD,23:C,4821,<seq>,<NAME>[,<arg>]*HH
$ACK,23:R,<seq>,<status>,<info>*HH
$ACK,23:C,<seq>,<status>,<info>*HH
```

`HH` is two hexadecimal digits: XOR of every ASCII byte between `$` and `*`, excluding both. Key `4821` is the existing shared numeric key, not cryptographic authentication. A sequence is decimal unsigned 32-bit (0..4294967295); GCS increments a random initial sequence modulo 2^32. The input frame is at most 99 bytes, with exactly one optional argument; printable non-space ASCII only, no embedded `$` or `*`, no empty fields. Firmware wire names/keywords are uppercase. Frames must match the selected vehicle; invalid checksum/address/framing is discarded without a machine ACK. Invalid sequence is discarded. R/C wrong keys return `REJECT,AUTH_KEY`; F returns `REJECT,KEY` (sequence 0). R/C do not replace a pending radio ACK to report a bad key.

Send LF after frames through the ESP32 receiver; CR or whitespace invalidates its line. It forwards unchanged downlinks and one queued uplink, validating the frame/address, with no automatic retries. Its `# UPLINK ...` diagnostics confirm transport only. The receiver does not authenticate the key itself. Its receive-window scheduling is separate per vehicle; idle fallback permits ON when routine telemetry is absent.

R/C direct Teensy USB accepts raw envelopes, LF/CR/CRLF, or local text `NAME,arg` / `NAME arg`. Local text is uppercased, assigned a separate USB sequence and wrapped in the same keyed envelope, then dispatched through the same handler. Raw envelopes are never repaired or re-keyed. Local line overflow/nonprintable bytes/partial lines older than 1 s are rejected. Baud setting is 115200. Plain USB `HELP` and `RESETCOUNT` are special local operations. Local-console access itself is not password-protected. F USB supports only the separate text commands listed below, not this Teensy envelope wrapper.

R/C/F immediate duplicate sequence: repeat cached ACK without repeating the action (same sequence with a different body also repeats the old ACK). A 16-entry RAM history rejects delayed duplicates with `REJECT,DELAYED_DUPLICATE`; it is not an unlimited or monotonic replay window. R/C restore the latest command sequence found in a checkpoint as `REJECT,RECOVERED_SEQ_OUTCOME_UNKNOWN`; the whole history and every command outcome are not durably recorded. F history resets on reboot. Use new sequences for deliberate new operations. No automatic retries are introduced.

Statuses are `OK` (accepted/commanded), `DONE` (operation completed to the extent reported), `FAIL` (execution/storage failure), `REJECT` (policy/configuration), `ERR` (bad argument/unknown command). There is no separate `$NACK` packet. ACKs route to the request's USB or LoRa connection, including delayed CAL/PAD/camera replies. ACKs and SYS health continue with routine telemetry OFF. An ACK never establishes physical servo position or camera recording.

**Existing global R/C restrictions:** while reboot is pending, only STATUS/PING/REBOOT reach their handlers (`REBOOT_PENDING`). While PAD reset is pending, only STATUS/PING (`PAD_RESET_PENDING`). While counter reset is pending, only STATUS/PING/TELEM,0 (`COUNTER_RESET_PENDING`). An outstanding radio ACK causes other radio commands to be dropped; direct USB replies use an independent path. These gates are unchanged and also apply to the new servo commands. No new test mode or flight-state acceptance policy was added.

**Existing GCS restrictions:** connect serial, stop simulation, and wait until no command is pending. Raw envelopes must match the page's address/key/checksum. One shared encoder/sequence/ACK timer handles all controls. Timeout, bridge error or disconnect reports execution unknown without retry. `RESETCOUNT` is explicitly blocked in both GCS consoles. CanSat GCS also blocks CAM/CAMPWR. These restrictions have not been loosened; any rejection is a follow-up issue, not something to bypass with a servo test.

## R/C command catalog

Manual radio commands (existing envelope, no schema change):

| Command | Results and restrictions |
| --- | --- |
| `RADIO` with exact MAIN/BACKUP | MAIN=865.50 MHz, BACKUP=867.25 MHz. ERR ARG_MAIN_OR_BACKUP / RADIO_UNAVAILABLE; REJECT RADIO_BUSY when TX/ACK pending. Existing global gates apply. While retune pending, all mutating commands reject RADIO_SWITCH_PENDING; STATUS/PING still work. Already-selected channel returns OK RADIO_CHANNEL without retuning. Otherwise OK RADIO_SWITCH_PENDING on old channel, then DONE RADIO_CHANNEL/MHZ/SF/SYNC on new channel, or FAIL RADIO_SWITCH_FAILED with LINK=UNKNOWN. Frequency only, no journal/mission/counter reset or extra IMU/state gate. Five-second service deadline; no retry. |
| `STATUS` with RADIO | OK current RADIO_CHANNEL, MHZ, SF, SYNC and PENDING, without changing hardware. |
| `GROUND` MAIN/BACKUP/STATUS | GCS console only: sends local receiver `@RADIO MAIN/BACKUP/STATUS`, never an addressed LoRa command. Requires live serial, no simulation or pending command; invalid argument sends nothing. Receiver configuration messages are not flight ACKs. Raw framed GROUND to flight is UNKNOWN. |

Receiver tracks only valid-key RADIO requests and follows an exact matching accepted ACK after checking checksum/vehicle/sequence/channel. ACKs remain forwarded unchanged. Both maintained GCS consoles keep RADIO acceptance pending until DONE/FAIL/timeout and block GROUND commands during that wait; no automatic retries. Receiver status messages do not confirm the flight computer's channel. Local ground-console commands are trusted USB operations, not new remotely executable unauthenticated flight commands. Reset defaults to MAIN; no automatic channel hopping. Lost ACK may separate flight/receiver and needs GROUND recovery. Full operation/failure policy: [MANUAL_RADIO_SWITCH.md](MANUAL_RADIO_SWITCH.md).

In this table, `W(NAME,arg)` means the exact full envelope above with the indicated NAME and optional argument, correct R/C address, fresh sequence and calculated checksum. It is notation, not a command sent on the wire. Unless noted, malformed required arguments return ERR and no hardware effect. Runtime settings have no new persistence across reset.

| Command and exact payload after sequence | Target | Arguments/defaults and purpose | Results, restrictions, effects | GCS control |
| --- | --- | --- | --- | --- |
| `HELP` — `W(HELP)` | R,C | No argument; supported-command summary | OK; read only; global gates apply to framed form. Plain local USB HELP prints additional hints before those gates. | Console |
| `PING` — `W(PING)` | R,C | No argument; link check | OK with RSSI/SNR or `USB_OK;RADIO_UNAVAILABLE`; no mission/hardware changes. | Console |
| `STATUS` — `W(STATUS)` | R,C | No argument; default health/state | OK `ST`, `TX`, `RATE`, camera request (R) or PSMS state (C), sensors, pressure reference, counter/calibration state. Unknown STATUS argument currently falls back to default status, not ERR. Read only. | Automatic on connect; console |
| `STATUS,LINK` — `W(STATUS,LINK)` | R,C | USB/radio diagnostics | OK `LINK=USB_AND_LORA;RADIO=...;RX=...;TX_BUSY=...;BUILD=USB_CMD_2`; read only. | Console |
| `STATUS,SD` — `W(STATUS,SD)` | R,C | SD file and health | OK `SD`, `FILE`, written/dropped rows, slow operations; read only. | Console |
| `STATUS,IMU` — `W(STATUS,IMU)` | R,C | IMU diagnostics | OK initialization/configuration, A/G/V freshness, accuracy, ages, dropped/decode/reset/config errors. No sensor reconfiguration from this query. | Console |
| `STATUS,COUNTER` — `W(STATUS,COUNTER)` | R,C | Counter query | OK `PACKET`, `NEXT`, `HIGH`, `EPOCH`, `TX`, `RESET`; no number consumed. USB/GCS text `COUNT` aliases this query. | Console |
| `TELEM` — `W(TELEM,0)` / `W(TELEM,1)` | R,C | Exact 0/1; default ON | OK `TELEM_OFF` / `TELEM_ON`. ON accepted regardless of flight/recovery state subject to global gates. OFF requires completed recovery and state outside ASCENT/COAST/APOGEE/DESCENT; `RECOVERY_SCANNING` / `TELEM_REQUIRED_IN_FLIGHT` otherwise. Acquisition/logging/mission clock continue. OFF freezes routine numbers/reservations; ON resumes; launch forces ON. ERR `ARG` for other values. | Telemetry button; console; local `TELEM OFF` / `TELEM ON` aliases |
| `RATE` — `W(RATE,<ms>)` | R,C | Decimal 400..10000 ms inclusive; default 500 | OK `RATE=<ms>`; ERR `ARG_UNSIGNED_MS` or `RANGE_400_10000`. Scheduling only; no counter/mission reset; RF jitter adds to nominal period. | Console |
| `CAL` — `W(CAL,BARO)` / `GYRO` / `ACCEL` / `ALL` | R,C | One exact keyword; ALL includes all three | OK `CAL_STARTED_3s`, then DONE/FAIL on same connection. Requires completed recovery, PAD, no launched mission, no calibration pending, fresh reliable IMU/gyro and stationary unit; BARO requires BMP. GYRO/ACCEL require existing baro reference. Rejections include `MISSION_ACTIVE_OR_NOT_ON_PAD`, `CAL_BUSY`, `IMU_STALE_OR_UNRELIABLE`, `KEEP_ROCKET_STILL` (R) / `KEEP_CanSat_STILL` (C), `CAL_BARO_FIRST`; ERR `ARG`/`NO_BMP`. Failure includes stale data, movement, pressure instability, fewer than 100 fresh requested samples or storage failure. DONE `APPLIED_AND_PERSISTED`, or `APPLIED_VOLATILE_STORAGE_UNAVAILABLE` when storage is absent. Changes calibration/reference and estimator; persistent calibration checkpoint when available; not a new mission or counter reset. | Calibrate sends ALL; console for individual sets |
| `TIME` — `W(TIME,<unix>)` | R,C | Decimal UTC seconds 1700000000..4102444799 | Requires RTC, completed recovery, no incomplete launched mission. OK `RTC_SET`; REJECT `MISSION_CLOCK_LOCKED`; ERR `NO_RTC`/`ARG`; FAIL `RTC_VERIFY_FAILED`. Writes DS3231, verifies readback, logs clock event; does not start/reset mission time. Existing handler has no separate PAD-only check beyond the mission lock. | Set Time uses current browser Unix time; console |
| `ARM` — `W(ARM)` / `W(ARM,CONFIRM)` | R,C | Optional CONFIRM required when recovery hold needs review | OK `MISSION_INITIALIZING` after queuing ARM, not an asynchronous DONE. Requires healthy usable recovery journal, known non-exhausted counter, no unfinished launched mission, PAD/LANDED, fresh reliable stationary IMU, valid baro/flight reference and no calibration. R additionally requires recording stopped/no camera work. Rejections identify `STORAGE_NOT_RECOVERABLE`, `COUNTER_LEGACY_BASELINE_REQUIRED`, `COUNTER_EXHAUSTED`, `REVIEW_RECOVERY_THEN_ARM_CONFIRM`, `ACTIVE_MISSION_RECOVERY_PRESERVED`, `MISSION_STATE_UNSAFE`, sensor/reference/busy conditions; ERR `ARG_CONFIRM_OR_NONE`; FAIL `ARM_NOT_PERSISTED`. Starts a new mission identity in PAD; detection waits for committed ARM and reacquired reference. Rocket clears the servo automatic latch only as part of this intentional new mission; no servo movement. Mission time starts only at launch. | Arm Mission; console |
| `RESETPAD` — `W(RESETPAD,CONFIRM)` | R,C | Exact CONFIRM | OK `PAD_RESET_PERSISTING`, then DONE `ST=0;PAD_RESET_DONE;UNARMED;MISSION=...` only after durable mission-reset record. Requires completed recovery, idle healthy journal, PAD or LANDED, no unfinished active PAD-recovery mission, no calibration/counter operation; R requires recording stopped/no camera work. ERR `USE_RESETPAD_CONFIRM` / `MISSION_ID_EXHAUSTED`; REJECT `STORAGE_BUSY_OR_NOT_RECOVERABLE`, `IN_FLIGHT_RESET_BLOCKED`, `ACTIVE_MISSION_RECOVERY_PRESERVED`, `CAL_OR_COUNTER_BUSY`, `STOP_RECORDING_BEFORE_PAD_RESET`; FAIL `PAD_RESET_NOT_QUEUED` or delayed `PAD_RESET_STORAGE_FAILED_REBOOT_TO_RESOLVE`. Clears mission/phase/timers/reference, and R servo auto latch; retains calibration, counters, logs and settings. Does not move servos. | Reset to PAD; console |
| `REBOOT` — `W(REBOOT)` | R,C | No argument | OK `REBOOTING`; requires completed recovery, unarmed PAD, no launched mission or calibration/camera work. REJECT `RECOVERY_SCANNING`, `REBOOT_REQUIRES_UNARMED_PAD`, `CAL_BUSY`/`CAM_BUSY`. Drains ACK/TX and SD/NOR queues before MCU restart, with 2 s fallback. Retains durable mission/calibration/counter records, not volatile configuration. | Console |
| `COUNTER` — `W(COUNTER,<upper-bound>)` | R,C | One-time legacy migration; unsigned last-used upper bound 1..4294967294, >= recovered high watermark | Only while counter baseline unknown, recovery complete, journal usable and no migration pending. OK `COUNTER_BASELINE_PERSISTING`, FAIL `COUNTER_STORAGE_FAILED`; REJECT `COUNTER_MIGRATION_NOT_AVAILABLE`; ERR `ARG_LAST_USED_UPPER_BOUND` / `COUNTER_RANGE`. Reserves a durable baseline, skipping all earlier numbers. This is not counter reset; no separate delayed DONE. Operator must know a bound across all earlier transmissions. | Console |

No-argument names HELP/PING/REBOOT and STATUS currently do not all reject an extra argument; this task does not tighten the existing parser/handler behavior. Use the documented forms.

### Rocket camera commands (existing)

| Payload | Arguments, restrictions and results | Hardware/storage effect and GCS |
| --- | --- | --- |
| `CAM` — `W(CAM,1)` / `W(CAM,0)` | Exact 1=start intent, 0=stop intent. Requires camera enabled, recovery complete, no camera pending; start on LANDED is rejected until a new mission. `CAM_DISABLED`, `ARG`, `NEW_MISSION_REQUIRED`, `CAM_BUSY`, `CAM_RECOVERY_OR_STORAGE_HOLD`, `CAM_STORAGE_FAILED`, `CAM_UART_FAILED_OR_DISABLED` identify failures. Healthy storage: OK `CAM_PERSISTING`, then DONE `CAM_REC_START_SENT` / `CAM_REC_STOP_SENT` or FAIL `CAM_NOT_SENT_STORAGE_OR_UART`. Storage-unavailable bench path can return immediate OK sent. | Persists intent before UART where available and records results. Both intents currently send power-button operation 0x01 (`CC 01 01 E7`), with no camera-state feedback. GCS recording Start/Stop buttons preserve selected intent. |
| `CAMPWR` — `W(CAMPWR)` | No argument (ERR `ARG` otherwise). Rejects recording request, queued camera, incomplete recovery or launched mission with `CAM_RECORDING_REQUEST_ACTIVE`; OK `CAM_POWER_BUTTON_SENT` or FAIL `CAM_UART_FAILED_OR_DISABLED`. | One 0x01 click with intent/result logging; console only; no verified shutdown operation. |
| `STATUS,CAMERA` — `W(STATUS,CAMERA)` | OK diagnostic flags SR/ST/SF/TT/TF, landing timer, auto-stop state, estimator health; read only. CA=NA denotes absent physical feedback. Its ST field is not flight state. | Console; no actuation. CanSat has no corresponding camera implementation. |

Current refusal/ignore/timeout conditions across GCS, bridge and rocket are indexed by source function in [REJECTION_GUIDE.md](REJECTION_GUIDE.md). Flight-state IDs remain 0 PAD, 1 ASCENT, 2 COAST, 3 APOGEE, 4 DESCENT, 5 LANDED; the rocket enum now explicitly assigns and asserts all six values without changing command behavior.

Main SEDS BPHC GCS receives both known fleet downlinks (23:R/source1 and 23:C/source2) with independent sequence/epoch tracking. Receiving CanSat telemetry/SYS never changes rocket command target, telemetry control, onboard mission or phase markers. Commands and matching ACKs on this page remain 23:R only; the separate CanSat page retains 23:C-only commands. No shared/broadcast TELEM command was introduced. Both streams can populate existing charts and CSV on the main page when the ground serial bridge provides both.

Startup policy: fresh PAD boots on both maintained Teensy targets have routine telemetry OFF. Unsolicited SYS is also withheld until the first enabled telemetry session; requested STATUS and other command ACKs remain available before ON. TELEM,1 starts the existing scheduler without clearing packet/mission/storage state. Later TELEM,0 suppresses routine packets while SYS/ACKs remain available. Launch and interrupted in-flight recovery retain the existing forced-ON safety policy. GCS connect/reconnect requests STATUS only; the operator uses the existing telemetry button or console to enable transmission. It no longer sends an automatic OFF after connecting.

### Rocket servo commands

Output implementation: existing ServoControl.h now uses the bundled Teensy Servo library from servotestting/servotestting.ino (attach with 500..2500 us library bounds, then writeMicroseconds). Existing command names, accepted 1000..2000 us arguments, reply formats, ACK-before-output behavior, startup/deployment restrictions and pins are unchanged. No sweep, angle, attach or detach commands were added.


These reuse the exact common R envelope and all existing global gates. They do not add a new test mode or an additional PAD-only interlock. They are not implemented on CanSat/F. Do deliberate tests on an unloaded bench; firmware does not establish mechanical clearance.

| Command and exact wire form | Argument/default and result | Hardware/mission/storage effect; GCS |
| --- | --- | --- |
| `PWM4` — `$CMD,23:R,4821,<seq>,PWM4,<arg>*HH` | `<arg>` is `SPIN`, `STOP`, `REVERSE` (continuous PWM4), legacy `TEST90`, `SAFE`, or an unsigned integer pulse in configured inclusive limits, currently 1000..2000 us. PWM4 is user-confirmed continuous rotation: SPIN=1700 us, REVERSE=1300 us, STOP/SAFE=1500 us nominal neutral. Legacy TEST90=1500 us also commands neutral, not an angle. Direct USB: OK `PWM4_CMD_US=<us>;FEEDBACK=NONE`. LoRa: OK `PWM4_ACCEPTED_US=<us>;FEEDBACK=NONE`, then DONE `PWM4_CMD_US=<us>;FEEDBACK=NONE` after issuing PWM. | Commands only pin 22, continuously holding the last pulse at configured 50 Hz. Does not increment mission, change phase/time, clear/set deployment latch, or explicitly append flash/SD records. Existing later snapshots may include the last command sequence as usual. Existing GCS console or optional servo-commands.js snippet; no added buttons. |
| `PWM5` — `$CMD,23:R,4821,<seq>,PWM5,<arg>*HH` | PWM5 retains TEST90/SAFE/numeric arguments and rejects SPIN/STOP/REVERSE. Same independent pulse configuration; direct USB OK `PWM5_CMD_US=<us>;FEEDBACK=NONE`, or LoRa OK `PWM5_ACCEPTED_US=<us>;FEEDBACK=NONE` followed by DONE `PWM5_CMD_US=<us>;FEEDBACK=NONE`. | Commands only pin 23. Same isolation; existing console or optional snippet. |
| `STATUS,SERVO` — `$CMD,23:R,4821,<seq>,STATUS,SERVO*HH` | OK `P4=<0/1>;U4=<us>;D4=<0/1>;P5=<0/1>;U5=<us>;D5=<0/1>;AUTO=<n>;INTENT=<0/1>;SENT=<0/1>;C4=<0/1>;C5=<0/1>;FEEDBACK=NONE` | Read only; existing console or optional snippet status(). P=valid manual profile; U=last pulse commanded this boot (0=no pulses); D=deployment configuration confirmed, not physical deployment; C=continuous-rotation configuration (currently C4=1, C5=0). |

Servo errors: missing/bad keyword, sign, fractional pulse or uint32 overflow -> `ERR,ARG_SPIN_REVERSE_STOP_SAFE_TEST90_OR_US` on continuous PWM4, or `ERR,ARG_SAFE_TEST90_OR_US` on PWM5. A numeric out-of-range value (including `90`, which is 90 microseconds, not degrees) -> `ERR,PWM4_RANGE_1000_2000_US` / PWM5 equivalent. Disabled/invalid configuration -> `REJECT,PWM4_UNCONFIGURED` / PWM5 equivalent. Existing global pending-operation rejections still take priority. Extra comma fields or malformed envelopes are discarded at the common parser, not fixed by servo code. Raw lowercase TEST90/SAFE is invalid; plain local/GCS text uppercases it.


Wireless PWM commands use the existing GCS command input -> encoded $CMD frame -> ESP32 SX1262 receiver -> rocket SX1262 -> authenticated dispatcher. No direct rocket USB connection is required to operate them. For accepted radio PWM requests only, the rocket validates/queues the pulse without starting it, transmits the matching acceptance ACK, waits for that TX to finish using the existing radio airtime/completion mechanism, then issues the configured PWM and sends DONE over LoRa. GCS retains the same pending sequence after the acceptance ACK until DONE/FAIL. A startTransmit failure for that acceptance produces FAIL `PWM4_ACK_OR_OUTPUT_FAILED` / PWM5 equivalent without issuing PWM. Failed argument/configuration checks still return their existing ERR/REJECT immediately. Direct USB PWM behavior remains immediate. No new flight-state rejection, automatic retry, RF settings, address, key, checksum or packet schema changes were introduced.

The initial ACK confirms acceptance, not movement. DONE confirms the PWM API call, not physical position. A reset during output start can still prevent DONE; this change does not establish or repair servo power integrity. `STATUS LINK` returns the existing BUILD=USB_CMD_2 plus `SERVO=ACK_FIRST_1` to identify this firmware wirelessly. Source changes must be installed on the rocket and the updated GCS page reloaded; the receiver firmware does not need a servo-specific change. No firmware OTA upload path exists in this repository. No USB Serial Monitor session is needed for normal servo operation.
AUTO codes: 0 IDLE, 1 PERSISTING intent, 2 COMMANDED to at least one configured deployment output, 3 UNCONFIGURED/no confirmed deployment output, 4 STORAGE_FAILED (including queue failure or 5 s commit timeout), 5 RECOVERY_INHIBITED. INTENT means the automatic attempt is reserved, not movement. SENT is an automatic command flag, historical on recovery; U is current-boot output state. Neither proves position. Manual testing changes U only, never the automatic flags.

Ready-to-use Teensy USB/GCS console examples:

```text
PWM4 TEST90
PWM5 TEST90
PWM4 1200
PWM4 TEST90
PWM4 SAFE
PWM5 SAFE
STATUS SERVO
```

Equivalent wire payload endings include `PWM4,TEST90`, `PWM5,TEST90`, `PWM4,SAFE`, `PWM5,SAFE`, and `STATUS,SERVO`; they always require the common envelope, sequence and calculated XOR.

Checksum-verified complete examples (use fresh sequences for later deliberate operations):

```text
$CMD,23:R,4821,101,PWM4,TEST90*51
$CMD,23:R,4821,102,PWM5,TEST90*53
$CMD,23:R,4821,103,PWM4,SAFE*5D
$CMD,23:R,4821,104,STATUS,SERVO*7C
```

PWM4 TEST90 is retained as a legacy 1500 us neutral command for the now-confirmed continuous servo; it does not rotate 90 degrees and can explain a test with no visible motion. PWM5 TEST90 retains its nominal positional center target. Both servo assignments are independently editable in `finalfsw/ServoControl.h`. Automatic `deploymentConfirmed` and startup `startupSafeConfirmed` remain false pending mechanical confirmation; manual tests work with the nominal profile without changing those settings. SAFE currently means test-neutral, not a verified safe flight linkage position.



### PWM4 continuous rotation (user-confirmed hardware)

Use these in the existing rocket GCS command input, waiting for DONE/FAIL after each:

```text
PWM4 SPIN
PWM4 STOP
PWM4 REVERSE
PWM5 1200
PWM5 SAFE
STATUS SERVO
```

Exact new wire arguments: `$CMD,23:R,4821,<seq>,PWM4,SPIN*HH`, `PWM4,STOP` or `PWM4,REVERSE` in the same envelope. Authentication, checksum, sequence/replay checks, global acceptance/rejection policy and two-stage radio ACK/DONE remain unchanged. SPIN holds the configured 1700 us pulse until another accepted PWM4 command; REVERSE holds 1300 us; STOP uses PWM4.safeUs (currently nominal 1500 us). These are configurable bounded test drive values, not calibrated RPM or a verified clockwise direction. Continuous rotation means pulse controls speed/direction, not shaft angle. There is no encoder or timed revolution calibration: no exact 360-degree-one-turn command or invented duration is implemented. No automatic stop timer was added; send STOP explicitly. Power/reset/loss-of-signal mechanical behavior remains unverified.

PWM5 retains its independent existing positional/manual pulse profile and receives no command when PWM4 spins/stops/reverses. It does not accept SPIN/STOP/REVERSE. Existing SAFE, TEST90 and 1000..2000 us numeric arguments remain available on both outputs; on PWM4 SAFE/TEST90 are nominal neutral rather than position targets. Trim PWM4.safeUs if the actual continuous servo creeps at nominal neutral. Only per-output hardware configuration changes; automatic deployment confirmations remain false and manual spin never marks deployment or changes flight/mission state. STATUS SERVO adds C4/C5 configured-type flags; neither is physical feedback.

Optional snippet methods `servoCommands.spin()`, `servoCommands.stop()` and `servoCommands.reverse()` target PWM4 only. Existing test/safe/pulse/status methods remain available, including independent PWM5 control. No GCS layout/buttons were added. Install the updated rocket firmware to recognize these new arguments; updating the GCS/snippet alone does not change the device.
### Optional command script (no GCS layout changes)

The original rocket GCS layout is preserved; the existing ACK handler recognizes the two-stage wireless PWM result without new controls. There are no added servo buttons or panels. Use the existing command console directly, or open `servo-commands.js` as a browser DevTools Sources > Snippets snippet in the rocket GCS tab and run it once. It adds only `window.servoCommands` and sends nothing on load. Connect through the existing GCS, stop simulation and wait for each ACK. In DevTools Console:

```javascript
servoCommands.test(4)        // PWM4,TEST90
servoCommands.test(5)        // PWM5,TEST90
servoCommands.pulse(4, 1200) // PWM4,1200
servoCommands.safe(4)        // PWM4,SAFE
servoCommands.safe(5)        // PWM5,SAFE
servoCommands.status()       // STATUS,SERVO
```

These methods call the existing sendFlightCommand encoder/transport with unchanged key, sequence, checksum, pending-command guards and ACK display. They do not open another serial connection, retry automatically, add UI, or bypass restrictions. The snippet refuses to install on a CanSat page. Results appear in the existing command status/log; FEEDBACK=NONE means no physical position verification. Firmware validates configured pulse limits. No separate test GCS has been created; do not add test buttons to the main GCS without explicit user agreement.
### R/C local USB-only commands and aliases

- `RESETCOUNT CONFIRM` (or `RESETCOUNT,CONFIRM`): not a wire command. Requires unarmed PAD or completed LANDED, routine telemetry OFF, healthy idle journal, known counter, no pending reservation/migration/calibration/reboot/PAD reset/camera work/TX/ACK and an available journal sequence. Prints `# RESETCOUNT PENDING`, then `# RESETCOUNT DONE: count=0; next=1; telemetry remains OFF` after commit. Incorrect confirmation/restrictions print a reason; fault prints FAILED and blocks known sequence until recovery. Persists a new counter epoch; preserves logs/calibration/mission. GCS intentionally rejects RESETCOUNT; raw `$CMD,...,RESETCOUNT,...` remains unknown.
- `COUNT` is a local/GCS alias for `STATUS,COUNTER`, not a separate wire command. `TELEM ON` / `TELEM OFF` become TELEM,1/0. Plain HELP prints local hints; raw framed HELP returns an ACK. No Servo command bypasses the common dispatcher.

## Synthetic CanSat (F) commands

Wire identity is 23:C with the same envelope/key/checksum/sequence rules. All values and flight states are synthetic; no physical sensors, servo, camera, ARM, calibration, SD logging or real mission recovery exists.

| Wire name / payload after sequence | Behavior, arguments, results and effects |
| --- | --- |
| `PING` / PING | OK `FAKE=1;RSSI=...;SNR=...`; no hardware action. |
| `STATUS` / STATUS | OK synthetic ST/TX/RATE/FAKE/NVS/BACKOFF/NEXT; read only; optional argument is ignored. |
| `TELEM` / TELEM,0 or TELEM,1 | Exact 0/1; OK TELEM_OFF/ON; ERR ARG otherwise. No real flight-state restriction. Stops/resumes synthetic numbered data, not ACK/SYS. |
| `RATE` / RATE,400..10000 | Decimal inclusive range; default 500 ms; OK RATE=n or ERR RANGE_400_10000. Volatile scheduling only. |
| `RESETPAD` / RESETPAD,CONFIRM | Holds synthetic profile in PAD; DONE `ST=0;PAD_RESET_DONE;UNARMED;MISSION=1;FAKE=1`; ERR USE_RESETPAD_CONFIRM otherwise. No NVS counter reset or physical mission record. |
| `REBOOT` / REBOOT | No argument (ERR ARG otherwise); OK REBOOTING, restarts after ACK transmission with 2 s fallback. Preserves NVS reservation. While reboot pending only REBOOT/STATUS/PING accepted; others REJECT REBOOT_PENDING. |

Other valid keyed commands return `REJECT,SIMULATOR_UNSUPPORTED` (including PWM4/PWM5). The CanSat GCS can control supported wire commands; its calibration/arm controls are still real-target controls and are rejected by F.

F local USB text (115200, newline/CR, maximum 31 printable characters, case-sensitive except REBOOT/RESETPAD): `ON`, `OFF`, `STATUS`, `BACKOFF ON`, `BACKOFF OFF`, `FLIGHT`, `RESETPAD CONFIRM`, `REBOOT`. ON/OFF change synthetic TX; STATUS prints FAKE/TX/BACKOFF/NVS/NEXT/PERIOD; BACKOFF toggles random/peer scheduling only; FLIGHT restarts/resumes the synthetic profile without resetting packet numbers; RESETPAD holds PAD; REBOOT restarts ESP32. These print diagnostics, not authenticated ACKs; no key, checksum or sequence on this local path. Unknown input prints help; invalid/oversized lines are dropped. No servo support.

## Historical/reference sketches (not maintained flight targets)

`rocket_tx_teensy_cmd` retains its old unaddressed `$CMD,23,4821,<seq>,NAME[,arg]*HH` and `$ACK,23,<seq>,status,info*HH` protocol, same XOR, LoRa only. Current GCS/receiver deliberately rejects these legacy live addresses. Only the latest RAM sequence is cached; no durable mission/counter recovery. Existing commands: PING (OK RSSI/SNR), STATUS (OK health; ignores subtype), TELEM (first argument character 1 enables; 0 only PAD/LANDED; REJECT ONLY_ON_PAD_OR_LANDED), RATE (atol 400..10000; ERR RANGE), CAM (first char 1/0 sends old start/stop operations; OK CAM_REC_START/STOP; ERR ARG), CAMPWR (OK CAM_POWER_SENT), CAL (BARO/GYRO/ALL only; PAD and present requested sensor, no concurrent CAL; OK then DONE/FAIL; volatile calibration), TIME (Unix >=1700000000, RTC present, not inFlight; RTC_SET / NO_RTC / IN_FLIGHT / ARG), REBOOT (PAD only; REBOOTING / NOT_ON_PAD). It has no ARM/RESETPAD/COUNTER/servo commands. These historical weaker checks are not copied into maintained firmware.

`runcamreference/sketch_oct20a` accepts single lowercase characters from USB or the first character of a trimmed E32 Serial7 line: `p` power/photo button (0x01), `r` start/stop recording operation (0x03), `m` mode button (0x02), `w` Wi-Fi button (0x00), `i` device info request, `?` menu. UART camera packets use CRC8/DVB-S2. There is no keyed command envelope, sequence, ACK policy, flight restriction or journal in that reference. Its Serial2 camera/E32 wiring is different from the maintained SX1262 flight targets; do not apply it to them. Other sensor/radio/RTC reference sketches provide no matching operator command dispatcher.
