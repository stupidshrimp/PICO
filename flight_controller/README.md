# Feather Flight Controller

This directory is the **source of truth** for the Feather flight-controller
firmware. Firmware is developed here, in this repository, and flashed by opening
this folder in the Arduino IDE and uploading to the board.

It began as an import from
<https://github.com/stupidshrimp/Feather-Flight-Controller> at revision
`37b5ccddb71020155f3b18c4a9dc16a6a06221de9`, but has been developed here since;
that repository is history, not an upstream to sync with. Do not treat this
directory as a vendored copy and do not re-import over it — doing so would
discard the firmware work carried in this repository's own history.

## Layout

- `Main.ino` is the Arduino sketch entry point.
- `*.cpp` / `*.h` files at this directory level contain the flight-controller
  firmware modules.
- `tests/` holds host-compiled tests for the pure-math and pure-predicate
  headers. They build the exact flight code with no Arduino dependency and are
  run from the Python suite (`tests/test_*.py` at the repository root), so a
  logic change can be verified without a board.
- `references/` contains the upstream reference libraries and example code that
  were present in the flight-controller repository at the imported revision.

## Adding testable firmware logic

Anything that can be expressed as pure math or a pure predicate belongs in a
header with no Arduino include, so `tests/` can compile and verify it on the
host. `board_align.h`, `mag_cal_fit.h`, and `loiter_nav.h` follow this pattern;
`Main.ino` then includes the header and supplies the hardware and timing.
