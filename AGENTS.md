# Repository instructions

Preserve existing firmware, GCS features, authenticated command paths, safety interlocks, pin assignments, telemetry/storage formats and rocket/CanSat isolation. Read feature.md and commands.md before modifying behavior; use the current source to resolve older chronological repair notes.

Update commands.md in the same change whenever a command is added, removed, renamed, or changes format or behavior. Document only implemented commands, including restrictions and hardware/storage effects.

Update feature.md in the same change whenever architecture, hardware assignments, major features, safety behavior, build instructions or known limitations change. Distinguish source behavior and automated verification from physical hardware validation.

Run affected existing tests and build checks. Never infer physical actuation from an ACK, pulse command or simulation. Keep unconfirmed deployment mechanics isolated in configuration; do not silently invent safe positions or weaken command restrictions.

Preserve the existing main GCS layout. Do not add buttons or test panels without explicit user agreement. Servo testing uses the existing command console or optional servo-commands.js DevTools snippet; a separate test GCS may be considered only if requested later.

Keep commands.md very short: serial commands and essential argument ranges only. Detailed framing/ACK/restriction documentation lives in COMMAND_PROTOCOL.md and must stay synchronized with command behavior changes.
