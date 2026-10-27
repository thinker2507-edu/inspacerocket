# Quick help

Open **workswithoutcmd copy.html** in Chrome or Edge and connect to the ground receiver at **115200 baud**. It displays both vehicles. The existing Serial Write box sends commands to one vehicle at a time.

## Choose where commands go

| Type in Serial Write | Effect |
|---|---|
| `TARGET ROCKET` | Select rocket (default) |
| `TARGET CANSAT` | Select CanSat; camera controls are hidden |
| `STATUS` | Read the selected flight computer's state |
| `HELP` | Ask the selected firmware for help |

The vehicle status line shows the command target. Switching waits for pending commands to finish and queries STATUS when connected. It never broadcasts. Both telemetry streams remain visible. Telemetry status becomes UNKNOWN until the selected board replies.

## Everyday commands

| Command | What it does |
|---|---|
| `TELEM ON` / `TELEM OFF` | Enable/disable routine radio telemetry for the selected board; flight processing continues |
| `STATUS IMU` | Inspect acceleration/gyro availability |
| `STATUS INA` | Read voltage/current/power validity |
| `STATUS COUNTER` | Inspect packet numbering and storage blockers |
| `STATUS FLASH` | Read flash diagnostics; does not erase |
| `CAL ALL` | Request existing sensor calibration; keep stationary and wait for completion |
| `RATE 500` | Request telemetry period in ms (400–10000) |
| `REBOOT` | Request a restart, subject to firmware safety checks |

CanSat: `STATUS RTC`, `STATUS SPI`, `STATUS PSMS`. Rocket: use Start/Stop Recording buttons; both send the experimental 0x01 operation. An ACK does **not** confirm recording or physical actuation. No automatic command retries.

## Starting fresh — bench only

`RESETPAD CONFIRM` requests a new unarmed PAD mission while preserving logs and packet history. `ARM` begins the existing arming workflow; do not use casually during bench work. Firmware can reject these for safety or storage reasons.

Packet reset and flash erase are **not routine fixes**. `RESETCOUNT CONFIRM` is rocket direct USB only, telemetry OFF, with safety checks. CanSat `ERASEFLASH CONFIRM` is direct USB only in the first 60 seconds, with existing safety checks: it deletes all external flash history and calibration. See [commands.md](commands.md) before maintenance.

For exact rejection reasons and framing, see [COMMAND_PROTOCOL.md](COMMAND_PROTOCOL.md). For uploading the repository, see [GITHUB_SETUP.md](GITHUB_SETUP.md).
