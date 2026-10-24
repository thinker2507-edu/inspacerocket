# Packet count: USB reset and telemetry ON/OFF

Flash the updated sketch for the selected vehicle and open its updated GCS HTML.
The ground receiver firmware is unchanged: its existing raw CSV/SYS forwarding
preserves the counter information.

## Reset for a bench session

Connect directly to the flight Teensy's USB port (not the ground receiver).
Open Serial Monitor at 115200 baud with Newline or Both NL & CR. Wait for startup
recovery to finish, then send these commands individually:

```text
TELEM OFF
COUNT
RESETCOUNT CONFIRM
```

Wait for `# RESETCOUNT DONE: count=0; next=1; telemetry remains OFF`.
Then send:

```text
TELEM ON
```

The next routine packet is #1, then #2, #3, etc. `COUNT` reports the counter,
next number, reserved upper bound, counter epoch and telemetry state. The GCS
command console can query the same information with `STATUS,COUNTER`; the
destructive reset is intentionally available only over local USB.

A reset requires unarmed PAD, or a completed and unarmed LANDED mission;
telemetry OFF; finished recovery; a healthy, idle flash journal; and no pending
calibration, reservation, camera operation, ACK/TX or reboot. It never clears an
active/interrupted mission to make a counter reset possible. A transient busy
rejection can be retried manually once the pending work finishes. Authentication,
radio command sequence numbers and command replay checks are unchanged.

The journal commits `LOG_COUNTER_RESET` (type 13) before reporting DONE. Its
journal sequence is the new **counter epoch**. This is a new numbering namespace,
not an attempt to pretend packet #1 has never previously existed. Calibration,
mission data, camera state and existing SD/flash files are retained. No erase is
performed. A torn reset record leaves the old namespace in force; no reset DONE
is reported. A storage fault requires reboot recovery before numbered TX resumes.

## Telemetry ON/OFF

- OFF freezes routine telemetry numbering and the GCS RX count after any packet
  already in transmission finishes. ACKs, SYS diagnostics, sensor acquisition,
  mission processing and SD logging continue; ACK/SYS do not consume telemetry
  numbers. New packet ranges are not reserved while OFF.
- ON resumes from the next number in the current range; it does not reset any
  counter. Repeated ON/OFF commands are idempotent.
- Existing safety policy still rejects OFF during an active flight or while
  startup recovery has not established the flight state. Read the returned
  rejection instead of assuming OFF succeeded.

The GCS RX/TX/error display is a session statistic: RX counts accepted telemetry
frames, while TX counts serial command writes. Sending ON/OFF therefore adds to
the command TX statistic. It is not the vehicle's persistent telemetry packet ID.
The accepted vehicle packet ID/epoch is shown in vehicle communication status and
recorded in CSV (`packet`, `counter_epoch`). The RX statistic is reset when a new
counter epoch is confirmed; plots and recorded history are retained.

Normal MCU resets still preserve numbering. Reservations contain 64 numbers and
are committed before use; reboot skips the unused portion, so gaps are expected.
For example, reboot after packet #3 can resume at #65. ON/OFF without reboot does
not cause that gap. A reset followed by reboot while still OFF has no reserved
range and can still start at #1; reboot after the range was reserved skips it.

## Protocol compatibility

The 16 telemetry CSV fields retain their order. The existing optional source
field now carries `1;E=<epoch>` for rocket or `2;E=<epoch>` for CanSat. The XOR
checksum includes that text. SYS appends the epoch as an optional seventeenth
field. Updated GCS pages accept old packets/SYS as epoch zero until a newer epoch
has been seen. They reject older epochs thereafter, as well as duplicate/delayed
packet IDs within the current epoch. A newer epoch permits packet #1 again.

Use the updated GCS with counter resets; older GCS pages reject the lower packet
IDs. Do not downgrade to firmware that does not understand type-13 reset records
and expect the new reset namespace to survive recovery.

Verification: focused simulated telemetry OFF/ON scheduling, GCS epoch routing and
duplicate/corrupt/delayed packets, actual journal constant-evaluation tests for
reset commits/power cuts/calibration retention/reboot nonreuse, and ARM sketch
object compilation. These are software checks, not physical radio/SD/NOR tests.
