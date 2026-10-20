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

## Known-issue fixes, 2026-10-19

The four issues that `air_known_issues_test` tracked are fixed. For the last
three, the behavior is the one the firmware owner chose:

1. DISCHARGE faults DISCHARGE_FAIL if the DC link is still at 60 V or more 5 s
   after the loop opened (FSAE 2026 EV.7.2.2.c). The under-5 V-by-10 s check stays.
2. A shutdown node must read open for 3 ms (three samples) before SCC, PRECHARGE
   or TS_ACTIVE abort. A shorter glitch keeps the state and its commands.
3. In DISCHARGE, while all nodes read closed again, the loop's AIR is not judged
   and the discharge clocks restart when the loop opens. The driven AIR must
   still open within 100 ms, and only a TSMS cycle leads back to IDLE. A glitch
   or a re-closed e-stop is no longer reported as a weld.
4. IDLE faults TRACTIVE_VOLTAGE only at 6 V or more; INIT and DISCHARGE still
   need below 5 V to reach IDLE.

Each check moved from `air_known_issues_test.c` into `air_paths_test.c`, which
now has 31 tests, so the known-issues list is empty. `air_test.c` steps past the
debounce wherever it opens the loop, and checks the debounce boundary on all
seven inputs. In `air_invariant_test.c` the command rules now allow commands to
be held, but never turned on, during the 3 ms debounce. Its two known-issue
counters became rules, plus a third: no DISCHARGE_FAIL from a healthy discharge
circuit.

Run locally on Windows with Zig 0.14.1, for both `AIR_CONTROLLED_NEGATIVE` values:

- All four logic tests pass at `-O0`, at `-O2 -DNDEBUG`, and with
  `-fsanitize=undefined -fsanitize-trap=undefined`.
- `air_invariant_test` with the default seed: all 13 reachable fault codes,
  TS_ACTIVE entered 707 times, commands held through an open reading inside the
  debounce on 1,234 steps, and no rule broken. Eight more seeds (1, 2, 3, 42,
  1234567, 987654321, 0xC0FFEE and 0xDEADBEEF) also passed.
- Edge coverage via clang SanitizerCoverage at `-O0` with UBSan off: all 97 edges
  in `air.c` and `air_protocol.c` were reached, by `air_test` and
  `air_paths_test` alone. `air_test` by itself reaches 69. The same method
  reproduces the Stage 1 figures (90 of 90, and 66) on the previous commit.
- Mutation check: nine regressions were planted in copies of `air.c`, and each
  was caught in both topologies. They were: no 60 V check, the 60 V boundary
  moved, no debounce, a 10 ms debounce, the loop's AIR judged while the loop
  reads closed, the driven AIR left unchecked then, no 5 V / 6 V margin,
  contactor checks during a glitch, and deadlines timed from DISCHARGE entry.
  `air_invariant_test` alone still catches the four Stage 1 safety bugs.
- The five GPIO fake-HAL cases pass. `air.c` and `air_protocol.c` compile for
  Cortex-M4 with Zig's clang and `-Wall -Wextra -Werror`.

Not run: `bazel test`, `run_host_tests.py`, `check_protocol.py`, `coverage.sh`
(gcc and gcov), the GitHub workflow, and an `air.elf` build with the ARM GCC
toolchain. Nothing was flashed.
