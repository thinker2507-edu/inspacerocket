# Serial commands

Send these in the **rocket GCS command input** to control it wirelessly through the receiver. Wait for each command to finish. Direct Teensy USB also accepts them at 115200 baud with Newline/CRLF.

The main GCS displays both vehicles but commands only the rocket. Use the CanSat GCS for CanSat commands; nothing is broadcast to both.

## Rocket and CanSat

```text
HELP
PING
STATUS
STATUS LINK
STATUS SD
STATUS IMU
STATUS RADIO
RADIO MAIN
RADIO BACKUP
STATUS COUNTER
COUNT
TELEM ON
TELEM OFF
RATE 500
CAL BARO
CAL GYRO
CAL ACCEL
CAL ALL
TIME <UTC_Unix_seconds>
ARM
ARM CONFIRM
RESETPAD CONFIRM
REBOOT
COUNTER <last_used_upper_bound>
RESETCOUNT CONFIRM
```

RATE: 400–10000 ms. TIME: 1700000000–4102444799. COUNTER is legacy migration only. RESETCOUNT is direct Teensy USB only and requires telemetry OFF plus existing safety checks.

Fresh PAD boot: telemetry OFF. Connect GCS, wait for STATUS, then use the existing telemetry button or `TELEM ON`. Reconnect only queries state. Launch/in-flight recovery retains forced-ON safety behavior.

## Rocket only

```text
CAM 1
CAM 0
CAMPWR
STATUS CAMERA
PWM4 SPIN
PWM4 STOP
PWM4 REVERSE
PWM4 TEST90
PWM5 TEST90
PWM4 SAFE
PWM5 SAFE
PWM4 1200
PWM5 1200
STATUS SERVO
```

PWM4 is continuous: SPIN=1700 µs, REVERSE=1300 µs, STOP=1500 µs nominal; it spins until stopped, not exactly one turn. PWM5 keeps independent pulse control. Pulses: 1000–2000 µs. SAFE/TEST90=1500 µs; on PWM4 this means neutral, not an angle. No position feedback; stop may need trimming.

## Ground receiver only (GCS console)

```text
GROUND MAIN
GROUND BACKUP
GROUND STATUS
```

Available in both maintained GCS consoles; blocked while a flight command is pending.

MAIN=865.50 MHz, BACKUP=867.25 MHz; SF7, sync0x95 unchanged. RADIO switches the addressed flight computer and coordinated receiver; GROUND tunes/queries only the receiver. Raw receiver USB equivalent: `@RADIO MAIN`, `@RADIO BACKUP`, `@RADIO STATUS`. See [MANUAL_RADIO_SWITCH.md](MANUAL_RADIO_SWITCH.md).

## Fake CanSat ESP32 USB only

```text
ON
OFF
STATUS
BACKOFF ON
BACKOFF OFF
FLIGHT
RESETPAD CONFIRM
REBOOT
```

The GCS command console also accepts the Teensy command names except RESETCOUNT. A serial terminal on the LoRa receiver needs authenticated `$CMD` frames for flight commands; only its local `@RADIO` commands use plain text. See [COMMAND_PROTOCOL.md](COMMAND_PROTOCOL.md) for framing, responses and restrictions.
