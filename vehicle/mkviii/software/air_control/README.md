# AIR control on STM32G441KBT6

This C application ports the behavior of `vehicle/mkvii/software/air_control`
from `olin-electric-motorsports/olin-electric-motorsports`, branch
`ian/mkvii_structure`, commit `013a686ea2b3f662834e131348ce2f5f886ee802`,
into the new monorepo. It uses the monorepo's pinned STM32CubeG4 HAL and startup.

**Status: builds for STM32G441; pin assignments are intentionally unset.
This is an unvalidated port, not vehicle-ready firmware.** With the default
configuration it enters `AIR_FAULT_BOARD_CONFIG`, does not configure GPIO/CAN,
and cannot command contactors. No physical hardware was flashed or tested.

## Files to edit later

1. `air_board_config.c`: fill in `air_pins[]`. Each row has a GPIO port, pin mask,
   active level, pull resistor setting, and alternate function. `NULL, 0` means
   **unassigned**, not PA0. There are no guessed pin numbers.
2. `air_config.h`: confirm controlled contactor, millivolt thresholds, timeouts,
   and CAN identifiers. `AIR_CONTROLLED_NEGATIVE = 1` implements the assignment's
   AIR-minus control. Setting it to `0` selects the legacy AIR-plus topology.
3. `air_board_config.h`: confirm CAN bit timing and then set
   `AIR_BOARD_CONFIGURED` to `1` after the complete mapping and behavior review.

All non-LED rows are required. LEDs may remain `NULL, 0`. Validation rejects
missing required pins, duplicate pins, invalid polarity/pull settings, multi-bit
pin masks, unsupported GPIO ports, and PA13/PA14 (reserved for SWD). It does not
prove electrical suitability or that a pin is bonded out in LQFP32: check the
STM32G441KB datasheet and final schematic. Choose valid FDCAN1 RX/TX alternate
function pins; other GPIO functions do not require alternate-function routing.

### Pin table

All physical ports/pins are **TBD**. These identifiers index `air_pins[]`.

| Configuration identifier | Direction | Meaning when logically active | Legacy active level to verify |
| --- | --- | --- | --- |
| `AIR_PIN_MAIN_CTL` | Output | Assert driver for AIR-minus by default | High |
| `AIR_PIN_PRECHARGE_CTL` | Output | Request precharge from precharge/discharge board | High |
| `AIR_PIN_SS_TSMS` | Input | Tractive System Master Switch / final shutdown node closed | Low |
| `AIR_PIN_SS_IMD_LATCH` | Input | IMD latch shutdown node closed | Low |
| `AIR_PIN_SS_MPC` | Input | Legacy MPC shutdown node closed; confirm connector identity | Low |
| `AIR_PIN_SS_TSMP` | Input | Tractive System Measuring Point shutdown node closed | Low |
| `AIR_PIN_SS_HVD` | Input | High Voltage Disconnect shutdown node closed | Low |
| `AIR_PIN_SS_BMS` | Input | BMS shutdown node closed | Low |
| `AIR_PIN_SS_EMETER` | Input | Energy-meter shutdown node closed | Low |
| `AIR_PIN_IMD_SENSE` | Input | Insulation Monitoring Device reports healthy | High |
| `AIR_PIN_AIR_P_FEEDBACK` | Input | AIR-plus feedback indicates closed | High |
| `AIR_PIN_AIR_N_FEEDBACK` | Input | AIR-minus feedback indicates closed | High |
| `AIR_PIN_ERROR_LED` | Optional output | Latched firmware fault | High |
| `AIR_PIN_HEARTBEAT_LED` | Optional output | Main-loop heartbeat, toggled every 500 ms | High |
| `AIR_PIN_INIT_LED` | Optional output | Startup checks in progress | High |
| `AIR_PIN_CAN_RX` | Alternate input | Receive from CAN transceiver RXD | AF9 FDCAN1 |
| `AIR_PIN_CAN_TX` | Alternate output | Transmit to CAN transceiver TXD | AF9 FDCAN1 |

For example, if the schematic eventually puts TSMS on PB5, edit only its row:

```c
[AIR_PIN_SS_TSMS] = {GPIOB, GPIO_PIN_5, GPIO_PIN_RESET, GPIO_NOPULL, 0},
```

**PB5 above is an example, not an assigned or recommended pin.**
`GPIO_PIN_RESET` here means a low input represents a closed node. `GPIO_PIN_SET`
would mean a high input represents a closed node. For an output it means the
level that energizes its driver. Input pull settings default to `GPIO_NOPULL`
as in the old design and must match the new signal-conditioning circuitry.
Confirm MCU-side voltage levels and reset-state external pulls with the hardware
lead. Firmware cannot establish a pin level while the MCU is held in reset.

`AIR_PIN_MAIN_CTL` replaces the old `AIR_P_LSD` name. The assignment states that
AIR-plus is closed by shutdown-circuit power and this board closes AIR-minus
after precharge. Both feedback inputs retain their physical plus/minus meanings
regardless of `AIR_CONTROLLED_NEGATIVE`.

## Runtime behavior

- **INIT:** keep commands off; reject TSMS already closed (also prevents automatic
  re-arm after a watchdog reset); wait 4 s for IMD stabilization; allow another
  1 s for missing initial CAN messages; require fresh healthy measurements,
  both AIR feedbacks open, and tractive voltage below 5 V.
- **IDLE:** wait for the final shutdown node; require all seven monitored shutdown
  nodes closed before starting. Continue IMD, voltage, CAN and feedback checks.
- **SHUTDOWN_CIRCUIT_CLOSED:** allow 200 ms for the other, hardware-controlled AIR
  to close. With default topology this is AIR-plus. The controlled AIR must remain
  open. Then assert the precharge request.
- **PRECHARGE:** wait for measured tractive voltage to reach at least 95% of pack
  voltage before 5 s elapses. Then assert the controlled AIR. No fixed blocking
  delay and no mixed voltage units are used.
- **TS_ACTIVE:** retain precharge until controlled-AIR feedback confirms closure
  within 200 ms, then remove precharge. Loss of either feedback subsequently
  faults. TS_ACTIVE initially includes that brief closure-confirmation interval.
- **DISCHARGE:** any shutdown node opening during closure, precharge or active
  operation removes both software commands in the same state-machine step.
  After 100 ms verify both AIRs opened; require tractive voltage below 5 V within
  10 s. Wait for TSMS release before returning to IDLE. There is no separate
  discharge output; discharge is the external circuit's responsibility.
- **FAULT:** latch the first fault until reset, remove main/precharge commands,
  light the error LED, and keep attempting status messages when CAN was initialized.

The software commands **one** AIR and precharge; it cannot force the other AIR
open while its hardware coil circuit remains powered. The shutdown circuit must
directly remove coil power independently of software. Weld inputs are treated
as closed-contact feedback, not inherently as a welded condition: a weld fault
means closed feedback when the contacts should have opened.

### Timing, clock and interrupts

SysTick provides a 1 ms HAL timebase. The main loop samples every input and runs
the state machine once when the tick changes. All time comparisons use unsigned
elapsed time and tolerate the 32-bit tick wrapping. Each iteration consumes at
most three FDCAN FIFO entries. There are no blocking CAN waits or state-machine
delays. All data updates happen in the main loop, avoiding ISR/main-loop races.

Only the core SysTick interrupt is used. GPIO and FDCAN are polled, so this target
does not depend on peripheral interrupt entries missing from `common/startup.c`.
Do not enable EXTI/FDCAN IRQs without first supplying the full G441 vector table.
The polling design has a nominal 1 ms sampling period; actual worst-case response
time must be measured on the board. An IWDG watchdog (nominal 250 ms, LSI-dependent)
is refreshed only after a complete loop. HardFault and related exceptions remove
configured commands and stop refreshing it.

The target uses internal HSI16, with no assumed external crystal. FDCAN uses
PCLK1 at 16 MHz, divider 1, prescaler 2, segments 13 and 2, SJW 2: 500 kbit/s at
87.5% sample point. Validate bitrate and oscillator tolerance over expected
conditions. `air_stm32.c:clock_init()` is where an external-clock design would be
configured; change bit timing along with clock frequency. An external CAN
transceiver is required. If it has a standby/enable input, provide the board's
required hardware strap or add its actual control signal before integration.

`air.ld` uses 128 KiB flash and 22 KiB ordinary SRAM. The additional 10 KiB CCM
bank is separate and unused. This avoids placing the stack beyond SRAM using
the existing common linker's 32 KiB contiguous assumption.

## CAN compatibility

| Message | ID | Bytes | Format / use |
| --- | --- | --- | --- |
| `bms_core` | `0x010` | 7 | Little endian, state bits 0–1, fault bits 2–17, pack voltage bits 18–33 |
| `IVT_Msg_Result_U1` | `0x414` | 6 | Byte 0 = 1; byte 1 high nibble contains error flags; bytes 2–5 signed big-endian millivolts |
| `air_control_critical` | `0x00D` | 4 | Legacy fault/state and shutdown/feedback bits; sent every 63 ms |

BMS pack voltage has **0.0256 V/count** in the legacy YAML. The decoder converts
it to millivolts with rounding. IVT has **0.001 V/count**, already millivolts.
Both measurements must stay fresh (BMS <1000 ms, IVT <500 ms), including while
active and discharging. BMS fault/non-operating state, IVT error flags, negative
tractive voltage, bus-off, RX FIFO loss and TX enqueue failures cause a fault.
Malformed/remote/extended/FD frames do not refresh message age. Frame reception
timestamps prove arrival, not that a remote sensor produced a new measurement.

`air_protocol.c` explicitly packs/unpacks this small legacy interface because
the shared generated receive wrapper does not expose per-frame timestamps or
validate incoming format/length. Wire tests compare this codec with the local
DBC. Update the codec and schema together if the BMS interface changes.

`legacy_bms.yml` is the original BMS message definition, not BMS application code.
The local `air_dbc` combines it, `air.yml`, and the existing IVT DBC. It is not
added to the global MK.VIII DBC yet because the new BMS contract is unconfirmed.
Legacy state/fault numbers are preserved; new faults 14–16 are BOARD_CONFIG,
CONTACTOR_FEEDBACK and IVT_STATUS. Existing consumers must accept these additions.

## Build and test

Run from the repository root with the team's Bazel environment:

```sh
bazel build --config=m4 //vehicle/mkviii/software/air_control:air.elf
bazel build //vehicle/mkviii/software/air_control:air_dbc
bazel test //vehicle/mkviii/software/air_control:air_test
```

The logic test is a **host** test: do not pass `--config=m4` when running it.
It also runs without Bazel using a native C compiler:

```sh
cc -std=c11 -Wall -Wextra -Werror -I. \
  vehicle/mkviii/software/air_control/air.c \
  vehicle/mkviii/software/air_control/air_protocol.c \
  vehicle/mkviii/software/air_control/tests/air_test.c -o air_test
./air_test
```

Repeat with `-DAIR_CONTROLLED_NEGATIVE=0` to exercise the legacy topology.
Tests exercise normal and repeated cycles, all shutdown inputs, IMD, voltage
threshold boundaries, stale CAN, fault latching, missing/stuck feedback, discharge
timeout, startup interlocks, tick wraparound and known CAN byte vectors.

To run both topologies plus GPIO mapping/polarity/interlock tests automatically:

```sh
python vehicle/mkviii/software/air_control/tests/run_host_tests.py --compiler cc
python vehicle/mkviii/software/air_control/tests/check_protocol.py --compiler cc
```

The second script needs `requirements_lock.txt`'s Python packages and checks
1,536 deterministic randomized C codec cases against cantools and the generated
DBC. It also exercises the fix in `common/can_api/dbc_generator.py` for DBCs with
no node list. The host runner uses a GPIO-only fake header isolated under `tests/`;
firmware builds use the actual STM32 headers. On Windows a portable compiler may
be supplied as `--compiler path/to/zig.exe cc`. Bazel DBC generation may need
`--shell_executable="C:/Program Files/Git/bin/bash.exe"` if it selects WSL bash
on a machine with no Linux distribution installed.

After filling/reviewing the configuration, the usual SWD targets are available:
`air_initialize`, `air_flash`, and `air_debug` under this package. The old AVR
bootloader/image metadata and CAN updater have deliberately not been carried
over. STM32 flashing uses the repository's ST-Link/OpenOCD scripts.

## Remaining hardware work

Confirm the pin/polarity table and package, input electrical levels, output
drivers and reset pulls, CAN transceiver/bitrate/messages, contactor identity and
movement times, and voltage thresholds. Use a low-voltage fixture to verify each
signal and inject every fault. Measure shutdown latency and reset/watchdog
behavior, verify the real precharge voltage trace and relay overlap, then conduct
team-supervised vehicle integration. Host tests and successful linking cannot
validate these electrical and mechanical behaviors.

References: [STM32G441KB product and datasheet](https://www.st.com/en/microcontrollers-microprocessors/stm32g441kb.html),
[pinned STM32CubeG4](https://github.com/STMicroelectronics/STM32CubeG4/tree/1e6984e6cdeb9e5c01f5badbfd18e5cf4345d02f),
[original AIR source](https://github.com/olin-electric-motorsports/olin-electric-motorsports/tree/013a686ea2b3f662834e131348ce2f5f886ee802/vehicle/mkvii/software/air_control).
