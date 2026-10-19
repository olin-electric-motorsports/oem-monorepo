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

## Stage 1 host tests, 2026-10-19

Run locally on Windows with Zig 0.14.1 (`zig cc -std=c11 -Wall -Wextra -Werror`)
for both `AIR_CONTROLLED_NEGATIVE` values:

- `air_test`, `air_paths_test` (21 tests), `air_known_issues_test` and
  `air_invariant_test` (10 million steps) all passed at `-O0` and at
  `-O2 -DNDEBUG`, and again with `-fsanitize=undefined -fsanitize-trap=undefined`.
- `air_known_issues_test` reports all four known issues as still present.
- `air_invariant_test` reached all 13 fault codes `air.c` can raise, entered
  TS_ACTIVE 694 times, and no safety rule broke. It also counted 75 weld faults
  with no stuck contactor and 199 `TRACTIVE_VOLTAGE` faults caused by 0.3 V of
  measurement noise at the 5 V threshold, both known issues.
- With four safety bugs planted in a copy of `air.c` (precharge at 50%, no abort
  when a node opens, IMD ignored after startup, TSMS allowed at boot), the
  invariant test failed on each one.
- Edge coverage via clang SanitizerCoverage, since `gcov` was not available: all
  90 branch edges in `air.c` and `air_protocol.c` were reached, by `air_test`
  and `air_paths_test` alone. `air_test` by itself reached 66.
- The five GPIO fake-HAL cases still pass.

Not run locally, because this machine has no Linux toolchain or Python:
`bazel test`, `run_host_tests.py`, `check_protocol.py`, `coverage.sh` and the
GitHub workflow.
