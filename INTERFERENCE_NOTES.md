Current update: manual MAIN (865.50 MHz) / BACKUP (867.25 MHz) switching is implemented for maintained rocket/CanSat and the ground bridge; see [MANUAL_RADIO_SWITCH.md](MANUAL_RADIO_SWITCH.md). Both still default to MAIN, and one ground SX1262 receives only one channel at a time. This is an operator-selected backup channel, not automatic link-loss hopping or a completed separate-channel coexistence design. SF7/sync0x95 are unchanged.

# Rocket / CanSat isolation and shared-channel coexistence

## Findings from the actual targets

`finalfsw/finalfsw.ino` and `cansatfswfinal/cansatfswfinal.ino` use the same
SX1262/RadioLib configuration, also matched by `inspacelora/lora_rx/lora_rx.ino`:

| Parameter | Rocket | CanSat | Ground |
| --- | --- | --- | --- |
| Frequency | 865.5 MHz | 865.5 MHz | 865.5 MHz |
| Bandwidth | 125 kHz | 125 kHz | 125 kHz |
| Spreading factor | 7 | 7 | 7 |
| Coding-rate denominator | 5 (4/5) | 5 (4/5) | 5 (4/5) |
| Sync word | 0x95 | 0x95 | 0x95 |
| TX power | 14 dBm | 14 dBm | 14 dBm |
| Preamble | 8 symbols | 8 symbols | 8 symbols |
| Routine interval | 500 ms default, RATE 400..10000 ms | Same | No periodic uplink |
| SYS health interval | 5000 ms minimum | Same | Raw forwarding |

They **can collide**. Previously identical team/key and command sequence values
also allowed either flight board to execute the same command and either ACK to
satisfy a GCS request. IDs solve logical routing, not RF interference. Each radio
is half duplex. Two high-rate feeds plus ACKs can overload the shared channel;
for example, a calculated 160-byte LoRa packet at SF7/BW125/4/5, explicit header,
CRC16 and preamble 8 occupies about 261 ms. Two nominal 2 Hz feeds would require
over 100% airtime before health/command traffic. This is an illustrative airtime
calculation, not measured packet length or a physical throughput result. The
existing TX duration and 250 ms RX guard already limit the achieved rate.

## Implemented software protections

The addresses are **23:R** (rocket) and **23:C** (CanSat). Team remains 23 and
key remains 4821. The vehicle suffix is mandatory and covered by the existing
XOR. No broadcast address exists and unaddressed legacy commands fail closed.

```
$CMD,23:R,4821,<uint32 sequence>,<NAME>[,<arg>]*XX
$CMD,23:C,4821,<uint32 sequence>,<NAME>[,<arg>]*XX
$ACK,23:R,<sequence>,<status>,<info>*XX
$ACK,23:C,<sequence>,<status>,<info>*XX
$SYS,23:R,<existing remaining fields>*XX
$SYS,23:C,<existing remaining fields>*XX
23:R,<existing remaining 15 sensor CSV fields>*XX
23:C,<existing remaining 15 sensor CSV fields>*XX
```

Command/ACK/SYS field counts and checksum conventions are unchanged: XOR bytes
between `$` and `*`. Serial commands have LF termination; LF is not radio payload.
Telemetry keeps 16 fields, units and order; field 0 is now the explicit address
and its new XOR suffix covers the entire CSV body before `*`, without `$`.
Rocket field 15 is now source marker 1; CanSat field 15 is source marker 2.
Both use numeric state in field 14. Historical addressed/checksummed rocket
state-text packets are still accepted, but new firmware emits source IDs only.
Checksum is accidental-corruption detection, not cryptographic authentication.

- `CommandValidation.h::commandAddressMatches()` validates checksum, canonical
  team/vehicle and framing before either `handleCommand()` can do anything. The
  existing key/sequence/command/safety checks still run. Wrong destinations are
  ignored without producing competing ACKs. TELEM only changes its addressed
  board, with boot/flight/recovery restrictions preserved.
- Immediate command duplicates retain cached ACK behavior. A 16-entry in-RAM
  `CommandHistory` additionally rejects delayed duplicates after intervening
  commands, rather than repeating camera toggles or other side effects. The
  existing persistent last sequence remains checked after reset. This bounded
  cache is not a persistent all-history replay defense; the cleartext key/identity
  protocol is not protection against a malicious transmitter forging frames.
- Each GCS has a fixed, explicitly displayed selected vehicle. Rocket controls
  cannot address CanSat, including through console envelopes. CanSat still has
  no camera controls; its console also blocks camera command names. PSMS remains
  unconfigured and has no invented commands/readings. There is no live dropdown
  that can accidentally retarget a pending command: intentionally choose the
  appropriate HTML file. A serial port is owned by one GCS connection at a time.
- ACKs/SYS must match that page's exact address. Live telemetry requires address,
  matching source, checksum and an increasing uint32 packet ID. Unknown, corrupt,
  duplicated or delayed telemetry cannot update plots, mission display or CSV.
  A wrong-vehicle ACK with the same command sequence cannot finish a pending
  request or change its confirmed telemetry state. Delayed unmatched ACKs are
  ignored. SYS remains an unordered health snapshot; it has no added delivery
  guarantee or anti-replay sequence. No retry or physical-execution claim is added.
- Each page retains its live telemetry high-water mark across local display reset
  and serial reconnect. CSV replay uses its original path and cannot change the
  live high-water mark. Counter gaps after reset remain valid. Offline historical
  CSV replay remains possible; legacy *live* frames do not bypass identity checks.
- The shared receiver forwards machine-readable payloads unchanged. Its uplink
  receive-window tracking is now separate for R/C, so another vehicle's downlink
  does not falsely indicate that the target is listening. The existing 1200 ms
  idle fallback remains, allowing TELEM ON while periodic data is OFF. Human-mode
  packet counters are also separate, and ACK/SYS are not parsed as sensor CSV.

## Collision mitigation, not collision elimination

Both targets have their own local `RadioCoexistence.h`:

1. When routine telemetry or periodic health is due, add a non-blocking
   pseudo-random integer holdoff of **100..500 ms** using a small xorshift PRNG.
   Seed mixes the actual Teensy MAC fuse values, CPU cycle counter and vehicle
   identity. This avoids identical deterministic schedules; it is not security
   randomness. RATE remains a minimum due interval, not a guaranteed exact rate.
2. On reception of a checksum-valid peer telemetry/ACK/SYS packet, or a command
   addressed to the other vehicle, defer scheduled traffic for **250 ms**, then
   draw a fresh holdoff. This protects the peer's uplink/ACK opportunity when the
   peers can hear each other. Sensors/flash/mission logic continue throughout.
3. Existing receive-header protection still prevents scheduled downlinks during
   an incoming frame. Its timeout is now derived from `getTimeOnAir(255)+30 ms`,
   so a maximum-size peer downlink is not cut off by the former command-sized
   300 ms bound. Receive buffer now accommodates complete peer frames.
4. ACK priority is unchanged: no randomized holdoff or peer quiet gate is applied
   to command acknowledgements. Successful transmissions clear the pending
   scheduling holdoff. OFF still has zero routine sensor transmissions; essential
   status and ACKs remain possible. Fresh sensor payloads are built only when
   the scheduled opportunity is actually used; there is no stale payload queue.

This reduces persistent lockstep collisions and offered load. There is no clock
synchronization requirement, CAD dependency, extra library, command retry or
frequency change. Hidden transmitters, overlapping first transmissions, capture
and near/far effects, external interference and saturation can still cause loss.
No guaranteed maximum latency, fairness or delivery is claimed.

No channel separation has been implemented or specific new channel proposed.
If bench tests show inadequate delivery, decisions still needed are telemetry
rate/latency requirements and whether to use an explicitly coordinated schedule
or approved separate channels. Separate channels require verified module/antenna
coverage and operating authorization for the location, and a ground arrangement
that can receive both (one radio cannot continuously receive two frequencies).

## Hardware and persistent-state isolation

These are separate Teensy boards. Both retain local SPI11/12/13 with SX1262 NSS2
and W25Q64JV CS10; setup deselects both before initialization. Radio RESET6,
BUSY4, RXEN1/TXEN0 are unchanged. Each board independently owns Wire2 SDA25/SCL24
BMP581/BNO085, Wire SDA18/SCL19 DS3231, Wire1 SDA17/SCL16 INA219 and Serial7 GNSS.
Identical pin numbers on separate boards are not an electrical bus conflict.
Do not interpret this as permission to connect both MCU SPI buses together.

Rocket RunCam remains Serial5 TX20/RX21 at 115200; CanSat does not initialize that
UART. PSMS model/interface/wiring are still unknown. No pin was assigned to it.
No physical schematic resolving additional connections was supplied.

Each target uses only its local headers and configuration, with no includes of
the other flight project. RF/pins/schema changes to one target do not rewrite the
other. Matching protocol/helper copies are checked by tests so drift is visible;
the ground receiver intentionally shares the rocket-side validation header.
Build both after a deliberate protocol update. External Arduino libraries remain
common dependencies, not shared mutable flight state.

Packet reservations, command sequences and journal sequences remain distinct.
Existing RFL1 (rocket) / CFL1 (CanSat) signatures, CRC/commit records, calibration
records, mission launch references and 64-number reservations are unchanged.
No startup erase, mission reset or counter migration was introduced. Each flash
must remain with its own vehicle; the other signature is FOREIGN, never silently
reused. Reset burns unused reservations instead of reusing transmitted IDs, and
mission time remains launch-only with trusted RTC recovery or explicit uncertainty.
All prior calibration, altitude, telemetry OFF and camera one-shot repairs remain.

Code cannot eliminate shared-power rail droop, SPI electrical faults or RF coupling
between nearby antennas. Supply adequacy/noise during radio/flash/camera activity,
antenna placement/isolation, actual module settings and simultaneous-radio behavior
still need physical measurements with the assembled hardware.

## Deployment and focused verification

Upgrade **both flight sketches, the ground receiver and the matching GCS files
together**. Old unaddressed live protocol versions deliberately do not interoperate.
No existing flash needs to be erased or packet counter reset for this upgrade.

- Rocket: `finalfsw/finalfsw.ino` and root `workswithoutcmd copy.html`.
- CanSat: `cansatfswfinal/cansatfswfinal.ino` and `cansatfswfinal/gcs.html`.
- Ground: existing board with `inspacelora/lora_rx/lora_rx.ino`.

Run `powershell -NoProfile -ExecutionPolicy Bypass -File tests/verify.ps1`.
It checks both actual command address gates and both GCS scripts, interleaved
traffic, wrong-vehicle same-sequence ACKs, malformed/checksum-invalid and unknown
addresses, delayed/duplicated telemetry, console routing, control state and OFF/ON.
`isolation.cpp` checks actual constexpr identity/history/coexistence helpers,
bounded timing and rollover. `persistence.cpp` compiles against each actual journal
and simulates independent device stores/clocks/counters plus existing torn/corrupt
record recovery. RF settings/pins and separate journal signatures are asserted.
Both flight translation units compile to ARM objects with flash enabled/disabled;
ground syntax/API compiles against installed ESP32/RadioLib headers. These are not
fully linked uploadable images, a propagation model, or physical radio tests.

Bench acceptance still required: operate both actual radios together, confirm
vehicle-tagged packet reception and usable latency, send PING/STATUS then TELEM
OFF/ON to one vehicle while observing that only its routine traffic changes,
repeat for the other, and reset each board separately without erasing flash to
check its address/next packet range/mission recovery. Inspect raw tagged ACK/SYS
traffic and independent logs. Exercise camera toggles only deliberately on the
rocket; UART transmission still does not confirm physical recording.
