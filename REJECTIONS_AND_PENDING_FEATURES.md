# Rejections and remaining implementation report

Source review: 2026-10-25. Maintained rocket, CanSat, ESP32 ground bridge, both GCS pages, optional servo snippet and synthetic CanSat. This is a source-based inventory, not a log of rejections observed on your hardware. No command policy or production behavior was changed to produce it.

## 1. Main findings

- Most refusals are deliberate authentication, freshness, mission/recovery or busy-operation checks. Removing them would not repair a failed sensor, radio link or storage device.
- `TELEM ON` has no IMU-quality gate. `ARM` and every calibration variant, including `CAL BARO`, do. Global pending-operation gates can still reject ON.
- **Arm before requesting recording.** Rocket `ARM` rejects an active recording request; the earlier camera-first workflow cannot then arm this firmware. Wait for mission initialization/reference readiness before requesting recording. An ARM acceptance alone does not establish launch readiness.
- Automatic servo intent/persistence exists, but both deployment confirmations remain false. Automatic physical deployment is not enabled. Manual servo control is implemented separately.
- PSMS acquisition is a placeholder. Faster indexed NOR startup recovery is not implemented. Simultaneous separate-channel ground reception is not implemented.
- Camera Start/Stop both send `CC 01 01 E7`, an experimental power-button operation. Recording feedback and reliable recording semantics on the actual camera remain unverified.

Primary references: [command protocol](COMMAND_PROTOCOL.md), [existing rejection guide](REJECTION_GUIDE.md), [features](feature.md), [engineering verification](FSW_VERIFICATION_REPORT.md), [operator commands](commands.md). Exact source lines and hashes for this review are saved in [rejection audit](verification_results/rejection-audit.json). The audit also lists additional sensor/storage diagnostic and refusal candidate lines; not every error string is a command NACK.

## 2. Interpret the result correctly

| Result | Meaning / operator implication |
|---|---|
| GCS blocked or button disabled | That path sent nothing. Check connection, simulation, pending request and known telemetry state. |
| Frame dropped / ACK ignored | Invalid or mismatched traffic, or unavailable transport; often no reply is generated. |
| `REJECT` | A policy, authentication or sequence-history gate refused the request. |
| `ERR` | Bad/unsupported command or argument, unavailable device/feature or invalid configuration. |
| `FAIL` | Processing failed, possibly after initial acceptance. Inspect the reason and any partial effects before another deliberate command. |
| Timeout / disconnect / serial or radio error | Execution is unknown. A toggle or servo command may have executed; no blind retry. |
| `OK` / `DONE` | Acceptance/software completion. Neither proves recording, movement, deployment or remote receipt. |

## 3. GCS blocks and discarded packets

Sources: main [GCS](<workswithoutcmd copy.html>) and [CanSat GCS](cansatfswfinal/gcs.html): `sendConsoleCommand`, `sendFlightCommand`, `sendReceiverRadioCommand`, control-update functions, ACK/SYS handlers, `parsePacket`, `acceptCounterEpoch`, serial/replay functions.

| Path | Every applicable category of block / discard |
|---|---|
| Console framing | Raw envelope must use the page's command identity, correct key/XOR, valid decimal uint32 sequence and matching syntax. Plain text permits NAME plus at most one argument. Bad input sends nothing. Raw input is not repaired automatically. |
| Command arguments | Name must match `[A-Z][A-Z0-9_]*`; supplied argument must be nonempty printable nonspace ASCII without comma, `*` or `$`. Complete frame must not exceed 99 bytes. |
| Connection / serialization | Disconnected/not-ready serial, unwritable port, active simulation or another pending command blocks live sends. Web Serial unsupported, port-open cancellation/failure, another application holding the port, stream errors and disconnect interrupt delivery. |
| Telemetry button | Live ON/OFF waits for a known firmware state. Reconnect requests STATUS only. Simulation changes local display state, not live RF. Timeout makes current state unknown; it does not confirm the requested change. |
| Confirmation and simulation | ARM/PAD-reset confirmation canceled: nothing sent. ARM, PAD reset, calibration and setting RTC are blocked during replay. |
| Local-only counter reset | Plain `RESETCOUNT` is blocked by GCS; use direct Teensy USB. Raw framed RESETCOUNT reaches no corresponding flight dispatcher command and returns `ERR UNKNOWN`. |
| Receiver channel control | `GROUND` accepts only MAIN/BACKUP/STATUS, requires connected serial, no simulation and no pending command. Receiver messages do not prove the flight computer's channel. |
| Camera / servo separation | CanSat GCS refuses camera sends. The rocket servo snippet refuses the wrong page, channel other than numeric 4/5, and noninteger/negative/overflow pulse inputs; firmware enforces narrower limits. Camera buttons accept only start/stop intents. |
| ACK validation | Bad syntax/status, XOR, address or uint32 sequence ignored. An unmatched/late sequence does not finish the current request. Main GCS commands and ACKs remain rocket-only even while it receives CanSat telemetry. |
| Pending / timeout | One pending command, 10 seconds. CAL, persistent CAM/PAD reset, wireless servo and RADIO acceptance remain pending for final DONE/FAIL. Timeout/write failure/disconnect means execution unknown, with no automatic retry. |
| Bridge diagnostics | Uplink invalid/partial/oversized input, USB rejection and failed uplink TX start can fail the GCS request before the flight handler runs. `# RADIO` is receiver status rather than a flight ACK. |
| Routine telemetry validation | Missing/bad XOR; wrong field count (must be 16); unknown identity; address/source mismatch (23:R/source1 or 23:C/source2); unmapped state (must be 0..5); invalid uint32 packet number; malformed/duplicate epoch; duplicate/delayed sequence; stale epoch; parsing error. Main GCS tracks both identities independently; CanSat page accepts C only. |
| SYS validation | Bad syntax/checksum/address or fields outside supported 14..17 schema; invalid integer/range/boolean fields; incompatible time quality/elapsed value; invalid/stale epoch. Most invalid SYS messages are silently ignored. Peer C SYS cannot change main-page rocket command/mission state. |
| Display / replay | Invalid GPS coordinates or unavailable map omit marker updates; tile-key/network errors are independent of command acceptance. Empty/malformed/canceled CSV, no valid rows or no recorded data block import/export/replay. Plot exceptions are logged. These are not flight command REJECTs. |

Invalid sensor values are allowed to remain invalid/NaN where the schema supports them; they are not fabricated as zero. Cached numeric IMU telemetry does not guarantee trusted freshness for ARM/CAL.

## 4. Bridge and flight parser: failures before execution

Sources: [receiver](inspacelora/lora_rx/lora_rx.ino) `validUplink`, `serviceUplink`, downlink handling; each target's `CommandValidation.h`, `ConsoleCommand.h`, `handleCommand`, `pollRadioRx`, `serviceUsbConsole`.

- Authenticated frames use `$CMD,23:R|C,4821,<uint32-seq>,NAME[,arg]*HH`. XOR covers bytes between `$` and `*`. Identity must match the intended target. Invalid envelope/checksum/vehicle is dropped; direct USB can print a diagnostic instead of an ACK.
- Frames must meet the helper's 9..99-byte bounds, separator/field counts and nonempty-field rules, with exactly two hex checksum digits and no embedded framing/control/non-ASCII/space bytes. Ground-local `@RADIO ` has only its explicit separator exception.
- Bad decimal sequence is dropped. Wrong/invalid decimal key produces `REJECT AUTH_KEY` when a reply can be queued. Validation is not bypassed to restore command acceptance.
- A radio request arriving while the prior radio ACK is pending is dropped (`# CMD busy: previous radio ACK pending`). Direct USB replies use their own transport.
- The latest repeated command sequence replays the cached result without executing. A sequence in the older 16-entry RAM history returns `REJECT DELAYED_DUPLICATE`. A recovered latest sequence can return cached `REJECT RECOVERED_SEQ_OUTCOME_UNKNOWN`. This is not a durable ledger of every historical command, nor a globally increasing command-sequence requirement.
- Receiver serial input needs LF; CR/control/non-ASCII/overflow invalidates the line. A partial line older than 1000 ms is discarded through the next LF. Teensy USB accepts CR/LF/CRLF, max 99 printable characters; invalid/overflow/expired partial lines are discarded. Plain parser limits name to 23 and argument to 47 characters and allows one argument.
- Invalid RF length/read failure, RX rearm failure, TX-start failure, collision and differing tuned channels can prevent delivery without a command REJECT. Receiver waits for a receive window but has a bounded idle fallback so ON can be sent while downlink is OFF.

## 5. Complete maintained-firmware REJECT inventory

R=rocket; C=real CanSat. Unless R-only is stated, the condition exists on both. Sources: [rocket firmware](finalfsw/finalfsw.ino), [CanSat firmware](cansatfswfinal/cansatfswfinal.ino), `executeCommand` and `handleCommand`; servo-specific return in [ServoControl.h](finalfsw/ServoControl.h).

First failing gate wins. Global order is RADIO pending, reboot pending, PAD-reset pending, counter-reset pending, then command-specific checks. Only STATUS/PING bypass RADIO and PAD-reset gates. Reboot also permits REBOOT; counter reset also permits exact TELEM,0.

| Exact REJECT info | Trigger / affected operations | Appropriate response |
|---|---|---|
| `RADIO_SWITCH_PENDING` | Retune outstanding; mutating commands refused. | Wait for DONE/FAIL; on uncertain handover use documented receiver recovery. |
| `REBOOT_PENDING` | Shutdown/reboot pending. | Wait/reconnect; do not assume a missing ACK means no reset. |
| `PAD_RESET_PENDING` | Durable PAD reset or its result pending. | Wait for final result. |
| `COUNTER_RESET_PENDING` | USB counter-reset commit pending. | Wait; STATUS/PING/TELEM,0 remain allowed. |
| `AUTH_KEY` | Wrong or invalid key in an otherwise addressed request. | Correct envelope/key; do not weaken authentication. |
| `DELAYED_DUPLICATE` | Sequence is in recent older command history. | Inspect previous outcome; use a fresh sequence for a new deliberate action. |
| `RECOVERED_SEQ_OUTCOME_UNKNOWN` | Cached recovered latest command has no certain outcome. | Inspect hardware/status before deciding what to do; no automatic replay. |
| `RECOVERY_SCANNING` | TELEM OFF, CAL or REBOOT before recovery finishes. | Wait for recovery/status. ON has no command-specific scan rejection. |
| `TELEM_REQUIRED_IN_FLIGHT` | OFF during states 1..4. | Existing flight safety policy; OFF is available in permitted PAD/LANDED conditions. |
| `RADIO_BUSY` | RADIO request while TX/ACK busy. | Wait for a quiet command window. |
| `NEW_MISSION_REQUIRED` (R) | CAM start while LANDED. | Intentionally initialize/arm the next mission under existing guards. |
| `CAM_BUSY` (R) | CAM/REBOOT while camera queue/result work pending. | Wait for camera completion. |
| `CAM_RECOVERY_OR_STORAGE_HOLD` (R) | CAM before recovery, or launched mission with unusable journal. | Resolve recovery/storage; do not substitute unlogged inflight toggles. |
| `CAM_RECORDING_REQUEST_ACTIVE` (R) | CAMPWR while recording requested, camera queued, recovery incomplete or mission launched. | This power-button path is intentionally restricted; inspect status. |
| `MISSION_ACTIVE_OR_NOT_ON_PAD` | CAL outside PAD or after mission launch. | Calibrate only on a fresh stationary pad mission. |
| `CAL_BUSY` | CAL/ARM while calibration active/reporting/persisting; REBOOT while active/persisting. | Wait for calibration final status. |
| `IMU_STALE_OR_UNRELIABLE` | ARM/CAL lack trusted recent IMU/gyro; CAL also requires BNO available. | Inspect STATUS IMU ages/quality/errors and physical sensor operation. |
| `KEEP_ROCKET_STILL` (R), `KEEP_CanSat_STILL` (C) | ARM/CAL stationarity test fails. Exact CanSat spelling is mixed case. | Keep vehicle still; inspect biases and actual fresh readings. |
| `CAL_BARO_FIRST` | CAL GYRO/ACCEL without existing pressure reference. | Establish valid BARO reference or request ALL with valid sensors. |
| `MISSION_CLOCK_LOCKED` | TIME before recovery or during launched incomplete mission. | Preserve flight time; set RTC in permitted preparation state. |
| `REBOOT_REQUIRES_UNARMED_PAD` | REBOOT unless PAD, unarmed, not launched. | Do not reset an active mission to bypass this check. |
| `STORAGE_BUSY_OR_NOT_RECOVERABLE` | RESETPAD without completed recovery, idle healthy journal/no critical loss. | Inspect recovery/store and wait if merely busy. |
| `IN_FLIGHT_RESET_BLOCKED` | RESETPAD outside PAD/LANDED. | Existing protection for active flight. |
| `ACTIVE_MISSION_RECOVERY_PRESERVED` | ARM on incomplete launched mission; RESETPAD on incomplete launched mission recovered into PAD. | Review interrupted mission rather than clearing it. |
| `CAL_OR_COUNTER_BUSY` | RESETPAD during calibration/report/write or sequence/migration operation. | Wait for durable completion. |
| `STOP_RECORDING_BEFORE_PAD_RESET` (R) | Recording request or camera queue/result exists. | Deliberate manual stop, wait for its software result, then PAD reset if safe. |
| `STORAGE_NOT_RECOVERABLE` | ARM without complete recovery, usable journal and no critical loss. | Restore storage integrity; volatile calibration does not make ARM available. |
| `COUNTER_LEGACY_BASELINE_REQUIRED` | ARM when packet sequence baseline unknown. | Supply a genuinely known legacy upper bound using documented COUNTER workflow. |
| `COUNTER_EXHAUSTED` | ARM with no next sequence available. | No uint32 wrap/reuse; deliberate supported maintenance required. |
| `REVIEW_RECOVERY_THEN_ARM_CONFIRM` | ARM on recovery hold without CONFIRM. | Review recovery first; CONFIRM does not override an active launched mission or other gates. |
| `MISSION_STATE_UNSAFE` | ARM outside PAD/LANDED. | Use existing mission lifecycle. |
| `BARO_REFERENCE_NOT_READY` | ARM without valid reference and fresh flight data. | Wait for stable pad acquisition; inspect BMP/pressure/freshness. |
| `STOP_RECORDING_BEFORE_ARM` (R) | ARM with recording requested or camera queue/result pending. | Arm and finish pad initialization before requesting recording. |
| `COUNTER_MIGRATION_NOT_AVAILABLE` | COUNTER baseline already known, migration pending, recovery incomplete or journal unusable. | COUNTER is legacy migration, not a routine reset. |
| `PWM4_UNCONFIGURED` / `PWM5_UNCONFIGURED` (R) | Invalid/disabled manual servo configuration. | Verify that output's configured profile and actual hardware. This differs from automatic deployment being unconfirmed. |

### USB-only counter reset

`serviceUsbConsole` requires exact `RESETCOUNT CONFIRM`. Existing pending reset prints PENDING. Otherwise any of these refuses it: unfinished recovery; nonidle/unhealthy journal or critical loss; unknown counter or pending reservation/migration; calibration/report/storage work; reboot/PAD reset pending; rocket camera queue/result; neither PAD nor completed LANDED; armed or launched incomplete mission; telemetry ON; radio TX/ACK busy; exhausted/invalid journal sequence.

The aggregate message is `# RESETCOUNT REJECT: need unarmed PAD or completed LANDED, telemetry OFF, idle healthy recovery store, no active mission/calibration`. Missing CONFIRM is a USB parser rejection. Not-queued/storage-fault results are FAILED diagnostics, not wire REJECTs. Success commits a new counter epoch; logs, calibration and mission are retained. No automatic erasure is performed.

## 6. ERR and FAIL results that can look like rejection

Same `executeCommand` sources above, plus calibration/camera/persistence services and ServoControl.

| Operation | ERR / FAIL info and cause |
|---|---|
| Unknown name | `ERR UNKNOWN`, including unsupported rocket commands on real CanSat and raw framed RESETCOUNT. Unknown STATUS subtype currently falls back to general STATUS rather than ERR. |
| TELEM/CAM/CAMPWR/CAL/TIME | `ERR ARG` for unsupported arguments; CAM also `ERR CAM_DISABLED` if disabled; CAL `ERR NO_BMP`; TIME `ERR NO_RTC`. TIME range is 1700000000..4102444799 UTC Unix seconds. CAMPWR requires no argument. |
| RATE | `ERR ARG_UNSIGNED_MS` for nondecimal/overflow input; `ERR RANGE_400_10000` outside inclusive range. |
| RADIO | `ERR ARG_MAIN_OR_BACKUP`, `ERR RADIO_UNAVAILABLE`; delayed `FAIL RADIO_SWITCH_FAILED;...;LINK=UNKNOWN` on acceptance TX failure, service deadline or radio API failure. Best-effort old-channel restoration is not a transactional RF guarantee. |
| RESETPAD | `ERR USE_RESETPAD_CONFIRM`, `ERR MISSION_ID_EXHAUSTED`; `FAIL PAD_RESET_NOT_QUEUED`; delayed `FAIL PAD_RESET_STORAGE_FAILED_REBOOT_TO_RESOLVE`. Remains inhibited rather than assuming a failed reset completed. |
| ARM | `ERR ARG_CONFIRM_OR_NONE`; `FAIL ARM_NOT_PERSISTED` if durable ARM cannot queue. Mission initialization may have begun but launch stays unarmed. |
| COUNTER | `ERR ARG_LAST_USED_UPPER_BOUND`, `ERR COUNTER_RANGE` (zero, below recovered high, UINT32_MAX); `FAIL COUNTER_STORAGE_FAILED`. |
| Servo argument/config | PWM4 `ERR ARG_SPIN_REVERSE_STOP_SAFE_TEST90_OR_US`; PWM5 `ERR ARG_SAFE_TEST90_OR_US`; out-of-range `ERR PWMn_RANGE_1000_2000_US`. Signs/fractions/overflow invalid. PWM5 does not accept continuous-only keywords. Numeric 90 means 90 microseconds, not degrees. |
| Wireless servo completion | `FAIL PWMn_ACK_OR_OUTPUT_FAILED`; initial acceptance TX failure prevents output. Successful local TX followed by lost RF ACK can still lead to output; no blind retry. |
| Camera completion (R) | `FAIL CAM_STORAGE_FAILED`, `FAIL CAM_UART_FAILED_OR_DISABLED`; delayed `FAIL CAM_NOT_SENT_STORAGE_OR_UART`. UART success is not physical recording confirmation. |
| Calibration sampling | After acceptance: `FAIL SENSOR_STALE_OR_UNRELIABLE`, `MOVED_OR_MISSION_ACTIVE`, `PRESSURE_UNSTABLE`, `INSUFFICIENT_FRESH_BARO`, `INSUFFICIENT_FRESH_GYRO`, `INSUFFICIENT_FRESH_ACCEL`, `INVALID_CALIBRATION_OR_STORAGE_BUSY`. Required sets need >=100 distinct fresh samples over the 3-second calibration. Pressure spread equivalent >0.75 m fails. |
| Calibration commit | `FAIL CAL_STORAGE_FAILED_OLD_PARAMETERS_RETAINED` on unusable journal or >5-second commit wait. Invalid candidate/pending write fails; calibration must have finite pressure strictly 2000..120000 Pa, gyro biases each absolute <20 deg/s, vertical bias absolute <2 m/s². Valid application with unavailable flash can return `DONE APPLIED_VOLATILE_STORAGE_UNAVAILABLE`; that is explicitly not persisted calibration. |
| RTC set | `FAIL RTC_VERIFY_FAILED` if checked readback is invalid or not within the permitted 1-second window. RTC write may already have occurred. |

Some no-argument commands do not enforce absence of an extra argument consistently. This is a parser/handler limitation, not permission to invent new argument behavior. Firmware HELP is a short hint and is not a complete enumeration of implemented commands; use commands.md.

## 7. Sensor, automatic-action and storage inhibitors

These generally produce status/invalid data rather than command REJECT replies.

| Subsystem / source | Existing inhibition / limitation |
|---|---|
| IMU: `ImuSample.h`, `readIMU`, ARM/CAL | Duplicate timestamp/sequence, nonfinite/implausible sample or age outside -2..100 ms cannot refresh trusted sample. Reliable requires accuracy >=1; gravity norm 8.5..10.8 m/s²; projected acceleration must be valid. ARM/CAL require reliable recent gyro too. Telemetry cache intentionally differs from trusted inputs. |
| Stationarity | Accel magnitude must be 0.85..1.15 g and each corrected gyro axis absolute <5 deg/s. CAL requires absolute vertical acceleration <2 m/s²; ARM uses bias-corrected absolute vertical acceleration <0.5 m/s². These are strict comparisons in source. |
| Estimator: `FlightEstimate.h` | Rejects invalid dt (>0.1 s or nonpositive), implausible baro height/jumps, baro absence >250 ms, or nonfinite/unbounded state. Trusted flight freshness is <=100 ms. Sparse CSV rows are not equivalent to fresh 50 Hz sensor samples. |
| Launch / descent / landing: target `loop`, `apogeeDetected` | Launch requires committed ARM, reference/estimator/fresh data, no recovery hold/calibration, sustained launch criteria. Stale data cannot advance fresh-sample counters. APOGEE holds 1 second before DESCENT; this timer is not another sensor confirmation. LANDED requires sustained fresh stationary evidence and is terminal until intentional mission initialization. |
| Servo: `ServoControl::reserve/service/restore` | Only launched, recovered, uninhibited APOGEE(3)->DESCENT(4) can reserve automatic intent. No durable commit, no confirmed deployment profile, commit timeout/storage failure, prior intent or ambiguous recovery inhibits output. AUTO=0 IDLE,1 PERSISTING,2 COMMANDED,3 UNCONFIGURED,4 STORAGE_FAILED,5 RECOVERY_INHIBITED. Current deployment flags false. |
| Camera: `serviceCamera`, `LandingStopTimer` | Stop requires recording requested, confirmed LANDED + >=60000 ms, allowed recovery/storage path and unconsumed attempt. No repeat each loop, no GCS dependence and no blind recovered toggle replay. Failure/inhibition is diagnostic, not proof of camera state. |
| NOR: `FlightJournal.h`, `FlashHardware.h` | Status 0 UNAVAILABLE,1 SCANNING,2 READY,3 FAULT,4 FULL,5 FOREIGN; only READY usable. Invalid/incomplete records rejected during recovery; queues/full/fault/critical loss can inhibit ARM/checkpoints/counter reservations. No startup erase or automatic overwrite. |
| Packet reservations | Numbered telemetry cannot transmit until a usable sequence reservation is durable. Unknown/missing/faulted/exhausted baseline blocks numbering; reset skips reserved tails rather than reusing numbers. OFF consumes none; no wrap at uint32 exhaustion. |
| SD: `SdFlightLog.h` | MISSING/OPENING/READY/FAILED/CLOSED and drops/slow-operation diagnostics. Missing/full/failed card can lose routine records without turning NOR into a sample fallback. Synchronous card calls can stall; logging is not hard-real-time certified. |
| RTC / INA: `AuxSensors.h` | Invalid RTC oscillator/calendar/time is not trusted for recovery. INA failures invalidate only INA, retry later and age out validity after 1 second; no fake zero readings. These are not blanket command REJECTs. |
| PSMS: `PsmsSensor.h` (C) | Always UNCONFIGURED; no actual protocol/measurements yet. No camera substitute packets are emitted. |

## 8. Synthetic ESP32 differences

[datafakecansat](datafakecansat/datafakecansat.ino) is not the real CanSat FSW. Its valid-frame command handler can return `REJECT KEY`, `REJECT SEQ`, `REJECT DELAYED_DUPLICATE`, `REJECT REBOOT_PENDING`, or `REJECT SIMULATOR_UNSUPPORTED`; malformed outer frames may be dropped before those checks. Invalid TELEM/REBOOT arguments return `ERR ARG`; RATE returns `ERR RANGE_400_10000`; bad PAD confirmation returns `ERR USE_RESETPAD_CONFIRM`. It does not implement real ARM, calibration, PSMS, camera, servo, RADIO switching or mission recovery. NVS reservation failure blocks numbered synthetic packets. Local invalid/oversized input is dropped or prints help. Never run fake and real 23:C together: their on-air identity is intentionally the same.

## 9. Features still requiring implementation or an explicit design decision

Priority indicates consequence, not authorization to alter flight hardware or safety policy. Validation work in section 10 is listed separately so existing implemented features are not mislabeled missing.

| Priority / item | What exists | What remains / prerequisite | Source / acceptance evidence |
|---|---|---|---|
| P0 Rocket automatic deployment configuration | Durable descent intent, one-shot controller, manual PWM, recovery inhibition. | Define actual linkage safe/deploy behavior and PWM4 continuous rotation stop policy; confirm each output before enabling automatic deployment. Do not simply flip flags or invent pulses. | `finalfsw/ServoControl.h`; verification SW-10/F01. Actual descent/backup demonstration required. |
| P0 Independent backup recovery chain | Primary MCU software is not a certified independent backup. | Supply/approve COTS backup design and integrate isolated battery/switch/driver/actuation as required by checklist. Repository evidence is missing; physical absence is not established. | Checklist §5; verification F04. Hardware decision/integration, not just FSW code. |
| P1 Simultaneous rocket/CanSat separate-channel reception | Address isolation, bounded randomized scheduling/quiet windows, manual MAIN/BACKUP switching, dual GCS parsing. | Decide operational channel allocation and simultaneous ground receiver architecture; update coordinated routing/config only after agreement. One SX1262 receiver cannot listen to both frequencies simultaneously. | `RadioCoexistence.h`, receiver, F02/PS-38. IDs/backoff do not eliminate RF collisions. |
| P1 PSMS driver and data integration (C) | Clear nonblocking driver boundary, no invented readings. | Exact model, interface, voltage, pins, protocol, units, rate and validity rules; then acquisition, telemetry/CSV/schema/GCS integration and failure handling. | `cansatfswfinal/PsmsSensor.h`; F12/SW-24. BLOCKED on hardware details. |
| P1 Faster NOR startup recovery | Bounded asynchronous 4 KiB reads; committed journal recovery on every boot, no erase. | Design crash-safe indexed/checkpoint startup plus fallback scan, preserving counter reservations/calibration/mission recovery. Current code still scans full 8 MiB. No implemented errors-only or power-cycle skip policy. | `FlightJournal.h`, setup/servicePersistence; feature.md. Must test torn/corrupt index and reset cases. |
| P1 Actual-camera recording semantics | Both manual intents and local landed+60s stop send 0x01 once, with requested/unconfirmed states. | Physically determine whether 0x01 toggles recording or power on this model, required state/timing, and feedback availability. Implement model-specific changes only from evidence and authorization. | `RunCamControl.h`, `serviceCamera`; F08. No verified physical recording state today. |
| P1 Full flight/landing simulation coverage | Actual estimator/decision replay and separate timer/journal tests. | Add explicitly labeled realistic sensor-rate/noise/orientation/dropout and post-impact rest scenarios; CanSat release profile separately. Original CSV ends moving and cannot exercise actual LAND +60s chain. | `verification_results/replay-manifest.json`, F07/F08. Test coverage, not a new flight mode. |
| P2 Source/build consistency safeguards | Root maintained targets and recorded test/source hashes; previous conflicting paths repaired. | Prevent recurrence of partial reversions and wrong nested-source builds through an agreed repository/version/build workflow. | F14; resolution JSONs. No Git metadata currently available in workspace. |
| P2 Operator help completeness | Full commands.md and existing console. | Firmware HELP does not list all current RADIO/camera/continuous-servo forms. Consider synchronizing short hints with implemented inventory; no missing handler inferred solely from HELP. | `executeCommand(HELP)`, `serviceUsbConsole`. Documentation/ergonomics gap. |

The following are **not outstanding mandatory features** under the current design: automatic frequency hopping/retries, an exact one-revolution servo command, OTA flashing, a camera power UI, PSMS commands with unknown semantics, and full NOR erasure during boot. They are absent intentionally or were never established requirements; do not add them merely to clear this list. MAIN/BACKUP selection already exists and is manual. Full timed or guaranteed mechanical actuation cannot be inferred from a pulse command.

## 10. Implemented features still needing qualification or missing evidence

| Area | Remaining work / existing discrepancy |
|---|---|
| Reference chute/airframe design | Simulation final 10-second downward speeds 6.838..6.891 m/s exceed checklist 2..5 m/s. Resolve native model/mass/canopy/CD and physical descent; this is not fixed by altering a GCS label or flight threshold. |
| Altitude/fusion accuracy | Pressure formula max difference ~2.983 m; pressure+ideal-inertial replay max altitude error ~8.189 m. No supplied accuracy tolerance establishes a pass/fail accuracy threshold. Need native atmosphere settings, sensor specifications and controlled data. |
| Launch/landing thresholds | Logic and offline checks exist; no representative raw launch/release or stationary post-impact dataset establishes flight qualification. Values are compile-time target-local configuration, not validated merely by their existence. |
| Calibration/IMU | Fresh-sample mean, pressure reference, gyro offsets and projected vertical bias are implemented. Capture STATUS IMU freshness/quality/reset/bus/power data on actual hardware; verify calibration application and sensor failures without removing guards. |
| Camera autonomous stop | Onboard nonblocking >=60-second timer and at-most-once intent exist. Needs actual LAND through stop event with valid sensors/storage and tested camera semantics. A reset may intentionally suppress uncertain one-shot replay; this is not an exactly-once physical guarantee. |
| Servo/backup | Measure pulses, neutral, supply/signal compatibility, boot/reset behavior, unloaded and loaded linkage. Primary/backup independent demonstration and approved mechanism/video remain absent from evidence. Manual tests do not advance flight state or mark deployment. |
| RTC/mission recovery | Launch-only monotonic mission clock and bounded trusted-RTC recovery exist. Verify OSF/time validity, reset duration/time continuity and explicit uncertainty with physical RTC. GCS first-packet display clock differs intentionally from onboard launch time. |
| INA219 | Voltage/current/power/shunt acquisition, units, calibration assumption and failure isolation exist. Verify actual shunt/value/wiring against a meter and valid/stale reporting; SYS INA current/power are unavailable when routine telemetry OFF per existing reporting policy. |
| NOR/packet recovery | Commit markers, CRC, append-only recovery, calibration and reserved counters implemented/tested in simulation. Physically verify W25Q64JV EF4017/CS10/power, missing/full/foreign/corrupt/torn-write behavior and counter nonreuse. Routine logs remain on SD, recovery records on NOR. |
| SD logging latency/endurance | CSV/row CRC, queues, critical flushing and unique files implemented. Actual card stalls, power-loss tails, full/missing/error behavior and endurance need tests. No hard bounded latency guarantee from synchronous SD APIs. |
| Radio/GCS | Test simultaneous real radios, ACK loss, local retune failure/reset and GROUND recovery. Packet routing tests do not prove RF coexistence. Add user's MapTiler key and verify actual browser/map/network layout. |
| Firmware build/deployment | Prior checks compile ARM objects and receiver/fake API checks; not linked flashable images. Perform selected-board full IDE link/upload and bench commissioning for each target. |
| Official documents/inspection | Full mission specification beyond checklist, native model, completed vehicle sheet, backup wiring/approval/datasheets, current unedited deployment video, mechanical/power/endurance inspection and organizer NETID/angular-unit interpretation remain missing or ambiguous. |

The existing engineering report assesses 71 checks: 15 PASS, 2 FAIL, 17 PARTIAL, 13 BLOCKED, 24 NOT TESTED. Its two explicit checklist/model failures are separate-channel simultaneous reception and excessive modeled descent speed. Those counts mix checklist/software/data scope; they do not certify the vehicle.

## 11. How to diagnose a refusal without changing policy

1. Capture exact status/info, intended identity, command/argument, pending sequence and whether transport was USB or ground bridge. Distinguish no transmission, no reply, REJECT and delayed FAIL.
2. Query `STATUS`, `STATUS LINK`, `STATUS RADIO`, `STATUS IMU`, `STATUS COUNTER`, `STATUS SD`; rocket also `STATUS CAMERA` and `STATUS SERVO`. Read-only STATUS/PING remain available during global pending gates.
3. For ARM/CAL IMU errors, inspect recent trusted data and stationarity rather than cached graph values. For storage errors, inspect recovery/journal state; do not erase or silently reset active history.
4. Prepare the rocket mission before requesting recording: healthy completed recovery, stationary pad/reference, deliberate calibration as needed, ARM, then wait for initialization/reference readiness, then CAM 1. The experimental 0x01 operation still requires camera bench confirmation. Do not blindly stop/start merely to bypass an ARM refusal.
5. For radio-switch timeout, consult [manual recovery procedure](MANUAL_RADIO_SWITCH.md). Do not assume either endpoint stayed on its old channel and do not retry a physical action solely because the GCS timed out.

## 12. Verification basis and files changed for this report

This task adds this report and a static source/message inventory in verification_results, and links the report from feature.md. It changes no firmware, GCS, command, safety check, radio setting or storage schema. commands.md needs no command-behavior update.

Existing executed results are in [test-runs.json](verification_results/test-runs.json): all 10 final stages of the preceding conflict-resolution verification passed, including CSV analysis/replay, calibration-source tests, current core suite, both Teensy ARM object configurations, ESP32 receiver and synthetic CanSat checks, report generation and artifact QA. Earlier failures and repairs are documented there and in the engineering report. These are previous executed checks, not newly run hardware validation or linked builds for this documentation task. The new static audit records source hashes and whether those files still match the last verification inventory; new report links/reason coverage are checked separately.

New documentation checks passed: all 42 inspected implementation files match their last verification hashes; all 32 extracted distinct literal immediate `executeCommand` REJECT reasons appear in this report; relative evidence links resolve. The audit retains 243 candidate source lines covering broader errors, parser/GCS/bridge and automatic inhibitors. Dynamic servo errors and cached/parser/simulator refusals are covered by the manually reviewed sections above rather than counted as literal immediate returns. Reproduce using `node verification_results/rejection-audit.cjs` (or the installed VS Code Electron-as-Node fallback); see [audit log](verification_results/rejection-audit.log). No firmware rebuild was necessary for this documentation-only change.

No sensor, camera, servo, backup mechanism, radio coexistence, power-cut or flight hardware validation is claimed. Unverified physical features must not be labeled confirmed merely because existing software tests pass.
