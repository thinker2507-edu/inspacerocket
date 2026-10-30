# D-day GCS

Open [index.html](index.html) in Chrome/Edge and connect the existing receiver.
The nine graphs fill the left column as a3x3 grid, with the map and existing command/flight
controls on the right. A compact desktop header keeps the data area prominent: small
**SIM**, **RESET SIM**, **CONNECT** and **CSV** controls, mission time, latitude, longitude,
packet counts, GPS altitude and phase indicators. CONNECT becomes DISCONNECT while connected.
CSV downloads recorded data; SIM opens saved-CSV replay. Upload CSV remains removed.

Detailed camera, command, telemetry, vehicle, altimeter and connection/CSV/file/baud status
is in the slim bottom footer. Hover a status for its complete message; status nodes remain
keyboard focusable and command responses remain in the console. Desktop header/footer are
44px/35px; narrower screens wrap/stack. The existing selector controls command routing;
redundant target-selected messages are omitted. Original common GCS and firmware are unchanged.

COTS UART2 TX17 -> rocket Serial2 RX7 -> rocket telemetry -> radio receiver USB -> GCS.
The orange fourth altitude trace is COTS pad-relative altitude. The grouped information
block explicitly labels **Altimeter UART**, **Altitude** (metres above pad), and **Release latch**.
`A`/`HB`/`L` in the chart show the same altitude/heartbeat/latch values. FRESH means a valid
UART heartbeat was recently received by the rocket, not overall sensor or mechanical health.
Gaps indicate invalid/stale altitude. Stale rocket downlink/disconnection resets the displayed
live altimeter status to unknown. Older telemetry lacks these fields and shows UNKNOWN.

Select Rocket and enable telemetry for live plotting; `STATUS ALT` also reports HB, AV,
ALT_CM and LATCH in the existing command response even while routine telemetry is off.
Both matching D-day firmware builds must be installed. No firmware reflash is needed solely
for this GCS rearrangement; reload the HTML.

New optional rocket fields and CSV columns are documented in [COMMAND_PROTOCOL.md](../COMMAND_PROTOCOL.md).
Older telemetry/CSV files are accepted with unknown COTS status. For servo tests use the
existing console or optional local `servo-commands.js`; no buttons/test panels were added.
See [D-day guide](../D-day_fsw/README.md) for firmware, wiring, commands and rearm.
