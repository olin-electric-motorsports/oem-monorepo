# Validation performed

Validated locally on 2026-09-29, branch `StefanC/AIRFirmware`.

- Built `air.elf` with Bazel 8.5.0, the repository's ARM GNU toolchain, and its
  pinned STM32CubeG4 dependency. Default and `AIR_BOARD_CONFIGURED=1` compilation
  both passed. Pin assignments remained unset in both; no board was flashed.
- Rebuilt the final ELF using the default `AIR_BOARD_CONFIGURED=0` configuration.
  ARM size report: text 16616 bytes, data 12 bytes, bss/reservations 2684 bytes.
- Built the local `air_dbc` and existing `//vehicle/mkviii:mkviii` DBC targets.
  Windows required the Git Bash `--shell_executable` override documented in README.
- Ran all seven C logic/protocol test groups for AIR-minus and AIR-plus control
  using Zig 0.14.1's native C compiler, with `-std=c11 -Wall -Wextra -Werror`.
- Ran five GPIO fake-HAL cases: unconfigured defaults, configuration enabled with
  missing pins, complete simulated pins with configuration disabled, enabled
  simulated pins including active-low output, and duplicate pins. Verified no
  GPIO access for rejected configurations, inactive latches before output mode,
  normalized input polarity, and command removal.
- Compared 1,536 deterministic randomized BMS/IVT/status C codec cases against
  cantools 39.4.5 and the generated DBC. All passed. This also exercised the shared
  DBC generator's fix for an IVT DBC with no node list.
- `git diff --check` passed for tracked changes.

These results cover software compilation, simulated logic and message encoding.
They do not validate PCB pin assignments, electrical levels, real CAN timing,
interrupt latency, watchdog tolerance, contactor mechanics, precharge/discharge
hardware, or vehicle operation. Follow README's hardware integration steps.

Changes are local and uncommitted; no GitHub push, PR, or hardware flash was made.
