# Rocket and GCS rejection guide

Checked against current source on 2026-10-24. This documents implemented checks; it does not disable any. Scope: maintained rocket `finalfsw/`, rocket `workswithoutcmd copy.html`, optional `servo-commands.js`, and the intervening ESP32 bridge. CanSat has its own implementation and restrictions; do not copy rocket-only commands to it. Detailed wire protocol: [COMMAND_PROTOCOL.md](COMMAND_PROTOCOL.md). Short operator commands: [commands.md](commands.md).

Find the named function and exact message with `rg -n 'MESSAGE' finalfsw 'workswithoutcmd copy.html'`. Line numbers change as code changes. Changing this document alone changes no behavior. Keep authentication, vehicle routing, bounds, recovery and actuator interlocks intact when editing a particular policy.

## Numeric flight states

| Number | Firmware constant | GCS phase |
| --- | --- | --- |
| 0 | ST_PAD | PAD |
| 1 | ST_ASCENT | POWERED_ASCENT |
| 2 | ST_COAST | COAST |
| 3 | ST_APOGEE | APOGEE |
| 4 | ST_DESCENT | PARACHUTE_DESCENT |
| 5 | ST_LANDED | LANDED |

`finalfsw.ino`: explicit numeric `FlightState`, compile-time compatibility assertion, `wireState()`, `buildPacket()`, `executeCommand(STATUS)`. Telemetry field 14 (zero-based) and STATUS `ST=` are numeric already. Text names are human labels, not an additional state. GCS `INO_STATE_MAP` and `setPhase()` map those numbers; old CSV phase aliases are retained. No BOOT or PAD* state. Do not renumber existing IDs: persisted mission states and GCS parsing depend on them. Numeric IDs stabilize representation; correct transition conditions still determine flight safety.

## What a failure means

| Result | Meaning |
| --- | --- |
| GCS blocked / disabled | No command was sent by that path. |
| Packet discarded / ACK ignored | Validation or routing failed, or reply did not match the pending command. Often no negative ACK. |
| REJECT | Valid addressed command reached firmware but a policy/authentication/history check refused it. |
| ERR | Unknown command, invalid argument/configuration, or unavailable compiled feature/sensor. |
| FAIL | An accepted/requested operation failed, sometimes after initial OK. Some effects may already have occurred; see source. |
| TIMED OUT / link lost / write failed | Execution is unknown. Not proof of rejection or non-execution. No automatic retry. |
| OK / DONE | Acceptance or software completion only; never proof of camera recording, servo movement or deployment. |

## GCS command blocks

Startup clarification: fresh PAD firmware now boots telemetry OFF and sends no unsolicited SYS until telemetry is first enabled. GCS connect/reconnect requests STATUS only, then the existing button can request ON. It never sends automatic ON/OFF. Launch/in-flight recovery retains forced-ON behavior. After a session starts, ordinary OFF still allows SYS/ACK traffic; it is not a missing-command or rejection symptom.

All functions below are in `workswithoutcmd copy.html` unless stated otherwise.

| Source function | Block / condition |
| --- | --- |
| `sendConsoleCommand` | Raw `$CMD` must match envelope syntax, selected `23:R`, key 4821, uint32 sequence and XOR checksum. Otherwise `Invalid command envelope/team/key/checksum; nothing sent.` |
| `sendConsoleCommand` | Plain text must be NAME with at most one space/comma-separated argument. Invalid syntax: `Use NAME,arg or NAME arg; only one argument.` Plain text is uppercased; TELEM ON/OFF and COUNT are aliases. |
| `sendConsoleCommand` | Plain RESETCOUNT is USB-monitor-only and is blocked here. A raw/DevTools RESETCOUNT cannot reset firmware counters: it has no authenticated dispatcher handler and returns ERR UNKNOWN. |
| `sendFlightCommand` | Name must match `[A-Z][A-Z0-9_]*`; argument must be nonempty printable nonspace ASCII without comma, `*`, `$`. Invalid name/argument is not sent. |
| `sendFlightCommand` / `updateCameraControls` | Disconnected/not-ready serial, no writable port, active simulation, or another command pending blocks transmission/disables buttons. One command at a time. |
| `sendFlightCommand` | Framed command longer than 99 characters is not sent. |
| `toggleTelemetry` | Live toggle waits until actual firmware telemetry state is known. In simulation it changes replay display state only, not radio transmission. |
| `calibrateSensors`, `setTime`, `armMission`, `resetPadMission` | Simulation blocks live calibration, RTC set, ARM and PAD reset. Canceling ARM/PAD confirmation sends nothing. ARM uses CONFIRM only when SYS reports recovery hold. |
| `sendCameraAction` | Only `start`/`stop` actions accepted. Button selection is not delivery/physical-state confirmation. |
| `connectSerial`, `handleCmd`, `sendCmd`, `readSerial` | Unsupported Web Serial, canceled/failed port open, closed/missing/not-writable port, failed stream read/write or disconnect prevent/interrupt delivery. Another app holding the COM port can cause an open failure. |
| `sendFlightCommand` | 10-second pending timer expires without a matching final reply: execution unknown. CAL, CAM_PERSISTING, PAD reset and accepted wireless PWM wait for final DONE/FAIL; their acceptance ACK does not end the timer. |
| `handleBridgeStatus` | `# UPLINK invalid/partial`, `# USB rejected/partial/command too long`, or nonzero uplink TX-start result fails the pending request. These can be transport/parser failures rather than an executeCommand REJECT. |
| `cameraConnectionLost` | Pending request fails with execution unknown; current telemetry state becomes unknown. Previous confirmed/requested values remain distinct. |
| `servo-commands.js` | Wrong/missing rocket GCS context throws; channel must be exactly 4 or 5; numeric pulse must be an integer 0..UINT32_MAX. Firmware further enforces its narrower pulse limits. No command on snippet load. |

## GCS discarded downlinks and local display omissions

| Source function | Discard / omission |
| --- | --- |
| `handleCameraAck` | Bad ACK syntax/status, wrong checksum/address, sequence above uint32: `Invalid ACK ignored`. Valid but no matching pending sequence: `Unmatched/late ACK`; does not complete another request. |
| `parsePacket` | Missing/incorrect XOR checksum, unknown vehicle (both 23:R and 23:C accepted on the main GCS), field count other than 16, state outside mapped 0..5, address/source mismatch (rocket=1, CanSat=2), malformed/duplicate E= token, invalid uint32 packet sequence, duplicate/delayed sequence, or parsing exception. Invalid sensor numbers may instead be retained as NaN/invalid, not grounds to fabricate zeros. |
| `acceptCounterEpoch` | Epoch must be integer 0..UINT32_MAX and not less than that vehicle's current epoch. Rocket and CanSat have independent packet/epoch tracking on the main GCS; one vehicle's reset does not reset the other. New epoch resets that vehicle's received count; same-epoch packet numbers must increase. |
| `handleSystemPacket` | Bad SYS syntax/checksum/address; field count outside 14..17; required integer fields invalid; time quality outside 0..3; uncertain quality without NA elapsed; other elapsed not a safe nonnegative integer; recovery >4, flash status >5, invalid RTC/INA/armed booleans, drop count, auto-stop outside 0..4, telemetry boolean or sequence status outside 0..5; invalid/stale epoch. Most invalid SYS frames are silently ignored. |
| `updateMap` | No map, nonfinite/out-of-range coordinates, or 0,0: no marker update. Not a rejected flight command. Tile API/network errors can hide the basemap independently of commands. |
| `toggleSim`, `uploadCSV`, `downloadCSV` | Missing/canceled file, empty/malformed CSV, no valid replay rows, per-row parsing errors, or no recorded rows for download. Replay completion stops replay; it is not a rocket command rejection. Plot exceptions are logged per plot. |

## Bridge and firmware validation before command execution

`finalfsw/CommandValidation.h`: `validCommandFrame`, `commandAddressMatches`, `decimalU32`; `finalfsw.ino`: `handleCommand`, `pollRadioRx`, `serviceUsbConsole`. Bridge: `inspacelora/lora_rx/lora_rx.ino`, `validUplink` and serial/uplink service.

- Frames must be `$CMD,<canonical team:R|C>,<key>,<seq>,<name>[,<arg>]*HH`, 9..99 bytes, correct XOR over bytes between `$` and `*`, exactly two hex checksum digits, 4 or 5 comma separators, no empty fields, embedded `$`/`*`, spaces/control/non-ASCII bytes. Radio payload has no CR/LF. Address must match team 23 / vehicle R on rocket. Invalid frame/wrong vehicle is dropped; USB prints a rejection diagnostic. These are not automatically negative ACKs.
- Key and sequence must be decimal uint32; wrong/invalid key returns `REJECT AUTH_KEY` if USB or radio ACK slot is available. Bad sequence is discarded. Raw framed input is not repaired or uppercased.
- Radio command while a previous radio ACK is pending is dropped with `# CMD busy: previous radio ACK pending`. Direct USB has independent ACK delivery. Latest repeated sequence replays cached outcome without executing again. An older sequence in the 16-entry RAM history returns `REJECT DELAYED_DUPLICATE`. This is not a globally monotonic command sequence check. A recovered last command may return cached `REJECT RECOVERED_SEQ_OUTCOME_UNKNOWN`; it is not re-executed.
- Radio packets with invalid length/read errors do not reach the handler. RX polling/receive availability, TX busy and RF collisions can cause missing replies without REJECT. Bridge queueing is serialized; RF TX start/read/rearm failure is a transport failure, not proof the flight handler ran.
- Bridge expects LF-terminated raw authenticated commands. CR/control/non-ASCII or oversized lines fail; a >1000 ms partial line is discarded through the next LF. Teensy direct USB accepts CR/LF/CRLF, 99 printable characters maximum, discards overflow/control-byte lines and partial lines after >1000 ms. Its plain parser limits name to 23 and argument to 47 characters, exactly one argument, valid name/characters; oversized constructed wire frame is also rejected. Local HELP prints hints without dispatcher execution.

## Global firmware interlocks

Additional manual-radio interlock: while `radioSwitch.pending`, commands other than STATUS/PING return REJECT RADIO_SWITCH_PENDING. `RADIO MAIN/BACKUP` otherwise keeps existing global gates, validates its argument (ERR ARG_MAIN_OR_BACKUP), requires initialized radio (ERR RADIO_UNAVAILABLE) and idle local TX/ACK (REJECT RADIO_BUSY). Failed acceptance TX/deadline/retune/RX API returns delayed FAIL RADIO_SWITCH_FAILED; old-channel restoration is best effort. See MANUAL_RADIO_SWITCH.md. The GCS blocks receiver-only GROUND commands during pending requests, simulation or disconnected serial, and only accepts MAIN/BACKUP/STATUS. A radio switch timeout is execution UNKNOWN, not proof the flight stayed on MAIN. No automatic retry/search.

First checks in `executeCommand`; first failing check wins, even before command-specific validation:

| REJECT info | Exact policy |
| --- | --- |
| REBOOT_PENDING | While reboot pending, only STATUS, PING and REBOOT proceed. |
| PAD_RESET_PENDING | While PAD reset commit/result pending, only STATUS and PING proceed. |
| COUNTER_RESET_PENDING | While USB counter-reset commit pending, only STATUS, PING and exact TELEM,0 proceed. |

## Command-specific firmware refusals

All in `executeCommand` unless noted. State numbers below refer to the table above. Existing global gates still apply to every row.

| Command | Status / info and failing condition |
| --- | --- |
| TELEM,1 | No additional IMU/state/recovery restriction. Enables routine TX; global gates still apply. |
| TELEM,0 | REJECT RECOVERY_SCANNING before recoveryDone; REJECT TELEM_REQUIRED_IN_FLIGHT in states 1..4; ERR ARG unless exact 0/1. ACK/SYS, logging and flight logic continue when OFF. |
| RATE | ERR ARG_UNSIGNED_MS unless decimal uint32; ERR RANGE_400_10000 outside interval. |
| CAM | ERR CAM_DISABLED if compiled out; ERR ARG unless 0/1; REJECT NEW_MISSION_REQUIRED for start in state 5; CAM_BUSY when camera queue/result pending; CAM_RECOVERY_OR_STORAGE_HOLD before recovery or when mission launched and journal unusable. FAIL CAM_STORAGE_FAILED if intent cannot queue; FAIL CAM_UART_FAILED_OR_DISABLED for direct UART path failure. |
| CAMPWR | ERR ARG for any argument; REJECT CAM_RECORDING_REQUEST_ACTIVE if recording requested/queued, recovery incomplete or mission launched; FAIL CAM_UART_FAILED_OR_DISABLED if send helper fails. No explicit on/off guarantee. |
| CAL | ERR ARG unless BARO/GYRO/ACCEL/ALL. REJECT RECOVERY_SCANNING, MISSION_ACTIVE_OR_NOT_ON_PAD (state !=0 or mission launched), CAL_BUSY (active/report/storage pending), IMU_STALE_OR_UNRELIABLE, KEEP_ROCKET_STILL, CAL_BARO_FIRST (no BARO in mask and no reference). ERR NO_BMP when BARO requested but unavailable. Even CAL BARO currently requires IMU checks. |
| TIME | ERR NO_RTC if absent/disabled; REJECT MISSION_CLOCK_LOCKED before recovery or launched incomplete mission; ERR ARG unless decimal 1700000000..4102444799; FAIL RTC_VERIFY_FAILED if readback invalid/not within 1 s. RTC write may already have occurred before readback failure. |
| REBOOT | REJECT RECOVERY_SCANNING; REBOOT_REQUIRES_UNARMED_PAD unless state0, unarmed and never launched for this mission; CAM_BUSY for queued/result camera; CAL_BUSY for active/persisting calibration. A missing reboot reply is not proof a reset did not occur. |
| RESETPAD | ERR USE_RESETPAD_CONFIRM unless exact CONFIRM; REJECT STORAGE_BUSY_OR_NOT_RECOVERABLE unless recovery complete, journal idle, no criticalLost; IN_FLIGHT_RESET_BLOCKED unless state0/5; ACTIVE_MISSION_RECOVERY_PRESERVED for launched incomplete mission recovered in PAD; CAL_OR_COUNTER_BUSY for calibration active/report/persist or sequence/migration token; STOP_RECORDING_BEFORE_PAD_RESET for active/queued/result camera. ERR MISSION_ID_EXHAUSTED at UINT32_MAX. FAIL PAD_RESET_NOT_QUEUED on checkpoint queue failure. |
| ARM | REJECT STORAGE_NOT_RECOVERABLE unless recovery complete, usable journal and no criticalLost; COUNTER_LEGACY_BASELINE_REQUIRED if sequence unknown; COUNTER_EXHAUSTED if next sequence zero; REVIEW_RECOVERY_THEN_ARM_CONFIRM if recoveryHold and argument !=CONFIRM; ERR ARG_CONFIRM_OR_NONE for other nonempty argument; REJECT ACTIVE_MISSION_RECOVERY_PRESERVED for launched incomplete mission; MISSION_STATE_UNSAFE outside state0/5; IMU_STALE_OR_UNRELIABLE; KEEP_ROCKET_STILL; BARO_REFERENCE_NOT_READY without reference/fresh flight data; STOP_RECORDING_BEFORE_ARM for active/queued/result camera; CAL_BUSY for active/report/persist calibration. FAIL ARM_NOT_PERSISTED if ARM event cannot queue (mission has been initialized but is left unarmed). |
| COUNTER | REJECT COUNTER_MIGRATION_NOT_AVAILABLE if sequence already known, migration pending, recovery incomplete or journal unusable. ERR ARG_LAST_USED_UPPER_BOUND unless decimal uint32; ERR COUNTER_RANGE if 0, below current high or UINT32_MAX. FAIL COUNTER_STORAGE_FAILED if checkpoint cannot queue. This only migrates a legacy counter; not a reset. |
| PWM4 / PWM5 | `ServoControl.h`: ERR ARG_SPIN_REVERSE_STOP_SAFE_TEST90_OR_US for invalid PWM4 continuous keyword/numeric syntax; PWM5 uses ARG_SAFE_TEST90_OR_US. Numeric sign/fraction/overflow invalid. REJECT PWMn_UNCONFIGURED if row disabled/invalid. ERR PWMn_RANGE_1000_2000_US outside configured inclusive range. PWM5 does not accept SPIN/STOP/REVERSE. Manual PWM currently has no extra PAD/IMU/arming gate: do not assume one exists. |
| Unknown command | ERR UNKNOWN, including raw RESETCOUNT through the common dispatcher. |
| HELP / PING / STATUS | No further command-specific refusal after global gates. STATUS falls back to general status for an unrecognized subtype; it does not reject it. PING reports USB_OK;RADIO_UNAVAILABLE when radio initialization failed. |

## IMU and stationarity conditions

Source: `readIMU`, flight block in `loop`, `calibrationStationary`, `ImuSample.h`, vertical-acceleration helper. These gates apply to ARM/CAL, not TELEM ON or manual PWM.

- `ImuSample.accept`: ignores exact duplicate timestamp+sequence; requires finite/plausible report and timestamp age between -2 ms and +100 ms. `fresh` requires age <=100 ms. `reliable` additionally requires sensor accuracy >=1. Accel norm must be <15 g; gravity norm 8.5..10.8 m/s2; gyro axes finite after offsets. Invalid sample/timestamp cannot refresh trusted freshness.
- `imuFresh`: reliable accel + gravity, both <=100 ms, valid gravity-projected vertical acceleration. CAL also explicitly requires bnoOK. ARM/CAL require reliable gyro <=100 ms. Failure gives IMU_STALE_OR_UNRELIABLE, even if cached telemetry axes look numeric.
- Calibration stationary: 0.85 < accel magnitude/g <1.15; absolute vertical acceleration <2 m/s2; each corrected gyro axis <5 deg/s; fresh reliable inputs above. Otherwise KEEP_ROCKET_STILL.
- ARM stationary: same gravity-magnitude/gyro bounds, but absolute vertical acceleration minus configured bias <0.5 m/s2. Otherwise KEEP_ROCKET_STILL.
- Diagnostics: `STATUS IMU` A/G/V freshness, Q accuracy, AGE milliseconds, DROP/DEC/RST/ERR counters. `STATUS` AV/reference and storage/sequence fields help distinguish other interlocks. Raw displayed IMU readings deliberately bypass the flight-quality gate.

## Deferred failures after acceptance

| Source | Failure condition / result |
| --- | --- |
| `runCalibration` | FAIL SENSOR_STALE_OR_UNRELIABLE if IMU/gyro stale/unreliable or requested BARO stale/unseen; MOVED_OR_MISSION_ACTIVE if not PAD/launched/not stationary; PRESSURE_UNSTABLE if pressure spread corresponds to >0.75 m; after 3 s INSUFFICIENT_FRESH_BARO, INSUFFICIENT_FRESH_GYRO or INSUFFICIENT_FRESH_ACCEL for requested sets with <100 distinct fresh samples; INVALID_CALIBRATION_OR_STORAGE_BUSY if candidate/write cannot be accepted. |
| `FlightCalibration::valid`, `beginCalibrationWrite` | Finite pressure strictly 2000..120000 Pa; absolute gyro bias each <20 deg/s; vertical bias <2 m/s2. Existing pending write or invalid candidate fails. If journal unavailable, valid calibration is applied VOLATILE and reports DONE APPLIED_VOLATILE_STORAGE_UNAVAILABLE, not persisted success. |
| `serviceCalibrationStorage` | FAIL CAL_STORAGE_FAILED_OLD_PARAMETERS_RETAINED if journal unusable or >5 s without commit; active parameters are not replaced before durable success. |
| `serviceCamera` | Waits for durable intent; >5 s/storage failure or failed send results in FAIL CAM_NOT_SENT_STORAGE_OR_UART. Automatic stop sets failure/inhibit status without blindly retrying. Commit is checked before timeout failure, so a committed camera intent may still be sent. |
| `servicePersistence` PAD reset | FAIL PAD_RESET_STORAGE_FAILED_REBOOT_TO_RESOLVE on failed/timed-out commit; keeps unarmed recovery hold/fault instead of assuming reset completed. |
| `ServoRadioCommand`, `serviceWirelessServo` | Wireless manual PWM waits for acceptance ACK TX before output. Failed initial TX yields FAIL PWMn_ACK_OR_OUTPUT_FAILED with no output; output validation failure also yields FAIL. DONE means PWM API was invoked, not measured motion. A lost over-air ACK can still occur despite successful local TX and subsequent output. |
| `ServoControl::service` | Automatic deployment is not a manual REJECT: no confirmed per-output deployment configuration means UNCONFIGURED/no deployment. Queue/flash failure or >5 s commit wait means STORAGE_FAILED. Recovered prior intent/ambiguous recovery inhibits replay. No automatic servo deployment is enabled in current configuration. |

## USB-only RESETCOUNT restrictions

`serviceUsbConsole`: exact CONFIRM required; existing counterResetToken reports PENDING without queueing another. Otherwise refuses if **any** of:

- Recovery incomplete; journal not idle; criticalLost; sequence unknown; sequenceToken/migrationToken pending.
- Calibration pending/active/report; reboot pending; PAD reset token/result pending; camera queue/result pending.
- State not PAD and not completed LANDED; armed; launched incomplete mission.
- Telemetry ON; radio TX busy; radio ACK pending.
- Journal nextSequence zero or UINT32_MAX.

Prints aggregate `# RESETCOUNT REJECT: ...` rather than a wire ACK. Queue failure prints FAILED: not queued. Later storage fault prints FAILED: storage fault; reboot for recovery, and blocks sequence use until resolved. Durable success creates a new counter epoch and leaves telemetry OFF. Does not erase historical records/calibration.

## Flight transitions and automatic actions that can be inhibited

Launch thresholds now live in the target's local `LaunchConfig.h`: LAUNCH_ACCEL_G, LAUNCH_SAMPLES, LAUNCH_MIN_ALT_M, LAUNCH_MIN_VEL_MPS, LAUNCH_FALLBACK_ALT_M and LAUNCH_FALLBACK_VEL_MPS. Values below describe current defaults; changing them requires rebuilding that target. These are not the numeric state IDs and are not runtime commands. Arming/recovery/sensor checks remain in the flight loop.

These do not generate command REJECT replies. Source: `setState`, `loop`, `apogeeDetected`, `serviceCamera`, `ServoControl.h`.

- LANDED is terminal except intentional new PAD mission. PAD launch needs recovered/no-hold mission, armed with committed ARM token, estimator ready, no calibration persistence, fresh flight data/no active calibration. No accepted baro sample means launch counters cannot advance. Five powered samples plus height >3 m and rising, or five climb samples height >15 m and velocity >10 m/s, gate launch; no single noisy sample suffices.
- ASCENT stale data clears burnout/apogee counts. COAST/APOGEE detection requires repeated fresh accepted baro criteria; 5 samples >1.5 m below max with nonpositive velocity. APOGEE -> DESCENT occurs after existing 1000 ms hold. This does not add another sensor confirmation.
- Landing requires fresh stationary data, fused/baro height within 5 m of pad, both velocity estimates <1 m/s, 3 s and >=100 accepted samples with <=0.5 m baro spread. Violations restart landing confirmation.
- Automatic camera stop only after confirmed landing +60 s, recording previously requested, supported command and allowed storage/recovery path. One-shot intent is latched before an attempt; recovery never blindly replays a toggle. No GCS connection needed.
- Automatic servo attempt only on existing APOGEE(3)->DESCENT(4) transition, launched/recovered/no hold, no consumed/inhibited latch; durable intent before independently confirmed output. Current deployment confirmations remain false. Manual commands do not reset the latch.

## Editing and verification

For a policy change, edit the specific handler/helper above, update COMMAND_PROTOCOL.md and commands.md if command behavior changes, and feature.md for safety/architecture changes. Keep the short command list short. Test normal acceptance **and** the corresponding rejection/duplicate/reset paths; do not change team/key/checksum just to bypass a failure.

Existing checks: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/verify.ps1`. Tests include GCS handler/ACK/isolation, real dispatch simulations, numeric marker mapping, telemetry, calibration, servo interlocks, journal recovery, both Teensy ARM object configurations and receiver syntax/API. Simulated radio/PWM/sensors are not physical validation. No checks here prove deployment or recording.
