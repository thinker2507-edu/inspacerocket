# SEDS BPHC project handoff

Open **[index.html](index.html)** in a browser. Offline, searchable and printable to PDF; this is documentation, not another GCS. A ready-to-share [PDF copy](PROJECT_HANDBOOK.pdf) is included.

- All production, ground and bench command families.
- Source-linked failure inventory plus silent-drop and timeout explanations.
- Firmware uploads, external-flash recovery/reset, hardware/automation and limitations.
- Actual verification results, separate from physical hardware validation.

Reflash **both flight targets** for command-policy/remote-counter changes. Reload the common GCS. Current receiver firmware stays unchanged. **Do not erase external flash to install this update.**

JSON catalogs/source hashes included. Optional regeneration: `node project_handoff/build_handbook.cjs`. Reading needs no server or tools. Keep this folder in repository root for source links; the HTML text also reads if shared alone.
