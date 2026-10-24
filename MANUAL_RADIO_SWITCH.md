# Manual main/backup radio switching

Operator-specified channels: MAIN **865.50 MHz**, BACKUP **867.25 MHz**. Both keep **SF7**, sync **0x95**, BW125 kHz, CR4/5, TX power14 dBm and existing radio pins/TCXO/RF-switch configuration. Only the frequency changes at runtime. No link-loss trigger, automatic channel search, automatic retry or broadcast was added.

## Flash and operate

Flash the updated `finalfsw/finalfsw.ino` and `inspacelora/lora_rx/lora_rx.ino`. If using CanSat channel switching, also flash `cansatfswfinal/cansatfswfinal.ino`. Reload the corresponding GCS HTML. Fake CanSat has no RADIO implementation and is unchanged.

All three real targets boot on MAIN; channel selection is RAM-only and resets to MAIN on reboot. Mission, calibration, telemetry enable state and packet reservations are not reset by switching channels. Existing boot/recovery policies still apply when rebooting.

Use the existing GCS command input, with the flight link currently working:

```text
STATUS RADIO
RADIO BACKUP
```

Wait for DONE on 867.25 MHz. To return:

```text
RADIO MAIN
```

There are no added buttons. Current rocket GCS targets 23:R; CanSat GCS targets 23:C. Only that addressed flight radio changes. One SX1262 receiver listens to one frequency at a time: an unswitched second vehicle remains on its old channel and becomes unavailable through that receiver until you tune back or switch it separately. Backup selection does not provide simultaneous reception on two separate RF channels and does not resolve shared-channel collision risk.

## What happens

1. GCS sends the existing authenticated/checksummed `$CMD` envelope with `RADIO,MAIN` or `RADIO,BACKUP` and its own sequence.
2. Flight validates the command and returns OK `RADIO_SWITCH_PENDING=...` on the old channel. This is acceptance, not completion. Mutating commands are temporarily blocked while pending; STATUS/PING remain available. Existing reboot/PAD/counter interlocks remain authoritative. No new IMU/flight-phase gate is introduced.
3. Receiver follows only that outbound request's checksum-valid ACK with the exact vehicle, sequence, OK status and expected target channel. Wrong key/argument requests are not registered as switch requests; corrupt/foreign/late ACKs cannot retune it.
4. Flight waits for its acceptance TX to finish, then at least 250 ms without blocking the loop. Routine telemetry/SYS scheduling pauses during the pending retune, but sensors, mission, logging and command RX continue.
5. Flight uses the installed RadioLib `standby`, `setFrequency`, `startReceive` API, then sends DONE `RADIO_CHANNEL=...;MHZ=...;SF=7;SYNC=0x95` on the new channel. GCS waits for this final reply within its existing 10-second timeout. It reports receiver-only messages as unconfirmed flight link status.

Repeating an already-selected channel with a fresh sequence returns OK current channel without retuning. Repeating an existing command sequence uses existing cached-result/history rules and does not execute the switch twice. Direct flight USB supports the same RADIO/STATUS commands, but a separate ground receiver cannot observe a USB-only acceptance: tune the receiver separately with GROUND afterward.

## Failure and manual recovery

Successful local acceptance TX does not prove the receiver heard it. If that ACK is lost, flight may switch while the receiver remains on the old channel. A lost final DONE can also leave execution unknown. There is no automatic recovery handshake or transactional guarantee.

After the GCS pending request completes/fails/times out, use:

```text
GROUND BACKUP
STATUS RADIO
```

GROUND BACKUP is receiver-only and sends no flight command. STATUS RADIO, if acknowledged by the flight computer, establishes that the addressed link is working on that frequency. If it gets no reply, try GROUND MAIN then STATUS RADIO. Do not blindly repeat camera/servo commands to test connectivity. Raw receiver USB accepts `@RADIO BACKUP` / `@RADIO MAIN` / `@RADIO STATUS`; these are local-operator commands, not an authentication bypass into the flight computer.

Receiver prints `# RADIO RECEIVER=...;MHZ=...;SF=7;SYNC=0x95;RC=...;LINK=UNCONFIRMED` after a local/followed retune. This reports receiver configuration/API status, not flight reception. It stops watching an unanswered switch acceptance after six seconds without retuning automatically. GCS ordinary delivery timeout remains ten seconds.

Failed acceptance TX, or more than five seconds waiting for the flight-side retune service, changes no flight frequency and returns FAIL. A frequency/RX API failure attempts to restore the old local frequency/RX and returns FAIL `RADIO_SWITCH_FAILED;MHZ=...;RC=...;LINK=UNKNOWN`. Restoration is best effort, not proof that the radio recovered; the receiver may already have moved and need manual recovery. No repeated switch attempts are scheduled. MCU/receiver resets during a switch can also separate the channels; both default to MAIN after their own reset.

## Verification scope

`tests/radio-switch.cpp` evaluates actual C++ parser/timing policy for identity/key/checksum/sequence/argument validation, ACK ordering, failure, timeout and millis rollover. `tests/radio-switch.test.cjs` runs actual GCS command/ACK handlers and actual flight retune-service source with radio APIs substituted, including failure/restoration paths and receiver-only commands. Both Teensy sketches and receiver receive the existing compile/API checks. No hardware RF test or simultaneous-radio coexistence validation is implied.
