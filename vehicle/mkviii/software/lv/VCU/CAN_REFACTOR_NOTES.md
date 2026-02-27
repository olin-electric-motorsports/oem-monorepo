# MKVII VCU CAN Refactor Notes

## Purpose
This document explains what changed in the VCU CAN design, why those changes were made, and how to integrate the new design with your existing firmware stack.

The comparison is primarily between:
- Legacy direct-CAN VCU draft:
  - `vehicle/mkiii/software/lv/throttle/vcu.h`
  - `vehicle/mkiii/software/lv/throttle/vcu.c`
- New MKVII VCU draft:
  - `vehicle/mkvii/software/lv/vcu/vcu.h`
  - `vehicle/mkvii/software/lv/vcu/vcu.c`

It also references current MKVII AVR board patterns:
- `vehicle/mkvii/software/lv/throttle/throttle.c`
- `vehicle/mkvii/software/lv/bspd/bspd.c`
- `vehicle/mkvii/software/lv/throttle/throttle.yml`
- `vehicle/mkvii/software/lv/bspd/bspd.yml`

## Executive Summary
The CAN layer was changed from **hard-coded, in-module HAL CAN calls** to a **transport-agnostic interface using hooks/callbacks**.

In short:
- Old approach: VCU logic directly knows CAN handle, CAN header, CAN ID, DLC, and payload layout.
- New approach: VCU logic outputs semantic data (torque, status, debug) and input requests (RTD, inverter fault), while a separate integration layer handles CAN encoding/decoding.

This was done to make the control logic reusable, safer to evolve, easier to test, and easier to adapt to different CAN stacks (HAL bxCAN, HAL FDCAN, generated `can_api`, etc.).

## Detailed Change List

## 1) CAN ownership moved out of VCU core

### Old
`vcu_hw_s` contained `CAN_HandleTypeDef* hcan_powertrain`.

The VCU module performed direct send:
- Built `CAN_TxHeaderTypeDef`
- Packed raw payload bytes
- Called `HAL_CAN_AddTxMessage`

### New
`vcu_hw_s` has no CAN handle. Instead, `vcu_hooks_s` defines callbacks:
- `read_can_inputs`
- `publish_inverter_command`
- `publish_vcu_status`
- `publish_throttle_debug`
- `publish_bspd_debug`

### Why
- Prevents mixing control logic with bus-driver specifics.
- Avoids lock-in to one CAN peripheral API.
- Reduces accidental coupling to one message format during early bring-up.

### Impact
- Requires one "glue layer" file in your application to bridge between VCU and CAN stack.
- VCU core becomes cleaner and easier to reason about.

## 2) CAN input model became explicit and extensible

### Old
`vcu_step_1ms(bool ready_to_drive)` consumed one boolean argument.

### New
Inputs are grouped in `vcu_can_inputs_s`:
- `ready_to_drive`
- `inverter_fault_active`
- `inverter_enable_feedback`

Can be injected either by:
- `vcu_set_can_inputs(...)`, or
- `read_can_inputs` callback during `vcu_main_loop_iteration()`.

### Why
- Real CAN state is usually multi-signal, not one bit.
- Future additions (timeouts, torque-mode bits, HV state) can be added without redesigning every function signature.

### Impact
- Cleaner interface for future rules and safety logic.
- Better traceability for faults that depend on CAN feedback.

## 3) Message scheduling aligned to control architecture (1 ms / 10 ms)

### Old
Single function `vcu_send_torque_can()` was called externally whenever desired.

### New
Ticked model:
- `vcu_request_1ms_tick()`: control update path
- `vcu_request_10ms_tick()`: periodic publish path
- `vcu_main_loop_iteration()`: background service + CAN input + tick execution

10 ms step is where status/debug publishing is intended.

### Why
- Matches your flowchart (1 ms control, 10 ms CAN publish).
- Keeps CAN publish rate deterministic and decoupled from loop jitter.
- Easier to prove timing behavior for safety review.

### Impact
- Integration must drive both ticks from timers/ISRs.
- Better deterministic behavior once connected.

## 4) Multi-stream CAN publishing replaced single-frame output

### Old
Primary output focus was one torque command frame.

### New
API explicitly supports:
- inverter command stream (torque + enable)
- VCU status stream
- throttle debug stream
- BSPD debug stream

### Why
- Your project architecture already has separate status/debug channels in MKVII (`throttle.yml` and `bspd.yml` pattern).
- Better observability for calibration, tech inspection, and post-run debugging.

### Impact
- You need message definitions and packing for each stream in integration code.
- System becomes easier to debug and validate.

## 5) Direct CAN frame constants removed from control core

### Old
CAN ID/DLC were controlled by preprocessor constants inside VCU source.

### New
No CAN ID/DLC in core logic.

### Why
- CAN IDs, payload mapping, and DBC evolution are project-level concerns.
- Prevents firmware logic change every time CAN spec changes.

### Impact
- CAN schema ownership moves to integration layer / DBC-generated layer.

## 6) Fault logic now consumes CAN feedback states

### Old
Most gating centered on local throttle + brake plausibility.

### New
Fault decisions include remote states:
- inverter fault bit from CAN input
- inverter enable feedback (available for future plausibility checks)

### Why
- Real vehicle safety needs both local sensor truth and networked subsystem truth.
- Prepares for stronger plausibility and timeout handling.

### Impact
- Safer architecture, but requires reliable receive/update handling.

## 7) Background service hook allows bus-side housekeeping outside core

### Old
CAN-specific concerns and logic concerns were blended.

### New
`vcu_main_loop_iteration()` calls:
- watchdog service
- bootloader service
- read CAN inputs
- 1 ms / 10 ms steps

### Why
- Clear execution contract for application loop.
- Enables system-specific housekeeping without polluting control logic.

### Impact
- Integration point is centralized and easier to audit.

## Comparison Table

| Area | Legacy VCU CAN | New MKVII VCU CAN | Why It Changed |
|---|---|---|---|
| CAN driver ownership | VCU core owns HAL CAN handle | App integration owns transport | decouple control from transport |
| Tx API | `vcu_send_torque_can()` direct frame send | `publish_*` semantic callbacks | easier schema evolution |
| Rx API | one bool (`ready_to_drive`) | struct (`vcu_can_inputs_s`) | supports richer vehicle state |
| IDs/DLC/payload packing | in VCU core | outside VCU core | avoid hard-coded protocol |
| Timing | external call discipline | explicit 1 ms/10 ms tick model | deterministic scheduling |
| Debug channels | mostly torque path | status + throttle debug + BSPD debug hooks | observability and calibration |
| Portability | tied to one HAL CAN path | transport-agnostic | supports bxCAN/FDCAN/generated APIs |
| Testability | harder (requires CAN HAL mocking) | easy (function pointer stubs) | unit/integration testing |

## Why This Refactor Is Better for MKVII

## 1) Fits your board consolidation goal
VCU combines throttle and BSPD logic. This naturally increases signal count and fault interactions.
By making CAN I/O semantic and modular, complexity grows in the integration layer (where it belongs), not in control code.

## 2) Matches your existing software style evolution
Current MKVII AVR boards use generated CAN interfaces from YAML (`can_api.h`).
That workflow already separates "signal meaning" from low-level frame handling.
The new VCU shape follows the same philosophy.

## 3) Better safety-case story
For design review / rules discussion, it is easier to justify:
- deterministic control cadence
- explicit fault data paths
- clearer separation between decision logic and comms stack

## 4) Faster iteration on CAN schema
When IDs/signals change, only the glue layer should change.
The torque/fault logic remains stable.
This reduces regression risk.

## 5) Easier bring-up with unknowns
Your request explicitly allowed uncertain areas to be left open.
This architecture supports that: unknown CAN details remain TODO in glue code, while control structure is implemented and testable now.

## Tradeoffs and Costs

## 1) More integration code up front
You must implement callback functions and register them in `vcu_hooks_s`.

## 2) No immediate "plug-and-send"
Without glue callbacks, nothing is transmitted.
This is intentional, but it means the first integration pass is mandatory.

## 3) Responsibility split must be documented
Team members need to know what belongs in:
- VCU core
- CAN adaptation layer
- DBC/YAML definitions

If this boundary is not respected, architecture drift can happen.

## What Is Intentionally Not Final Yet

The refactor intentionally leaves protocol details outside core:
- exact CAN IDs
- payload bit layout
- timeout thresholds per incoming message
- counters/rolling counters/CRC requirements
- retry and bus-off recovery policy

These should be finalized in your CAN schema + integration file, not in `vcu.c`.

## Recommended Integration Pattern

## Step 1: Create a CAN adapter module
Example file names:
- `vehicle/mkvii/software/lv/vcu/vcu_can_adapter.c`
- `vehicle/mkvii/software/lv/vcu/vcu_can_adapter.h`

This module should:
- decode incoming frames into `vcu_can_inputs_s`
- encode outgoing semantic outputs into project CAN messages

## Step 2: Implement callback functions
You need at least:
- `read_can_inputs(...)`
- `publish_inverter_command(...)`
- `publish_vcu_status(...)`
- `publish_throttle_debug(...)`
- `publish_bspd_debug(...)`

## Step 3: Hook into timers
- Timer ISR 1 ms -> `vcu_request_1ms_tick()`
- Timer ISR 10 ms -> `vcu_request_10ms_tick()`

Main loop:
- poll CAN receive queue/mailboxes
- update adapter cache
- call `vcu_main_loop_iteration()`

## Step 4: Apply message timeout policy in adapter
For each subscribed frame, keep:
- last reception tick
- validity boolean

If timeout exceeded:
- clear/force safe values in `vcu_can_inputs_s`
- optionally raise adapter-level fault flag

## Step 5: Keep mapping logic out of VCU core
Do not re-introduce frame IDs or byte packing into `vcu.c`.
Keep it in adapter + schema files.

## Adapter Skeleton (Example)

```c
/* Pseudocode only */
static vcu_can_inputs_s g_can_inputs;

static void can_read_inputs(void* ctx, vcu_can_inputs_s* out) {
    (void)ctx;
    *out = g_can_inputs; /* already updated by CAN RX handlers */
}

static void can_publish_inverter_command(void* ctx, int16_t torque, bool enable) {
    (void)ctx;
    /* Pack into inverter command message and send via CAN backend */
}

static void can_publish_vcu_status(void* ctx, const vcu_state_s* state) {
    (void)ctx;
    /* Pack fault bits/mode/inputs for telemetry */
}

static const vcu_hooks_s hooks = {
    .user_ctx = NULL,
    .read_can_inputs = can_read_inputs,
    .publish_inverter_command = can_publish_inverter_command,
    .publish_vcu_status = can_publish_vcu_status,
    .publish_throttle_debug = can_publish_throttle_debug,
    .publish_bspd_debug = can_publish_bspd_debug,
};
```

## Validation Checklist for the New CAN Design

- [ ] 1 ms and 10 ms ticks are generated at correct rates.
- [ ] `read_can_inputs` uses timeout-safe defaults.
- [ ] inverter command publish rate is deterministic.
- [ ] VCU status/debug channels publish at expected rate.
- [ ] ready-to-drive transition behavior matches requirements.
- [ ] APPS implausibility and brake/throttle latch behavior still force zero torque.
- [ ] BSPD-related latching behavior can be observed on debug stream.
- [ ] Bus-off/restart behavior is defined in adapter layer.

## Post-Review Hardening Updates (Gemini Feedback Applied)

The following improvements were implemented in the MKVII VCU draft after review:

1. Non-blocking ADC integration path added:
- New hook: `read_adc_samples(...)`
- New struct: `vcu_adc_samples_s`
- Intended use: DMA/ISR-updated sampling cache
- Behavior: VCU consumes cached samples when provided, and only falls back to blocking HAL polling for bring-up.

2. APPS fault shutdown gating corrected:
- APPS out-of-range/mismatch bits can still be reported immediately for diagnostics.
- Torque shutdown now keys off `blocking_fault_bits`, where APPS-related shutdown is controlled by the persistence latch (`VCU_FAULT_APPS_TIMEOUT_LATCHED`), not a one-sample spike.

3. Brake/throttle implausibility trigger made safer:
- Trigger and clear now use `apps_high_permille` (max of the two APPS channels), which avoids false-safe behavior when one sensor fails low.

4. Inverter command publish rate made configurable:
- Added `inverter_command_period_ms` calibration field.
- Default set to 10 ms.
- Prevents accidental 1 kHz command publishing from overloading bus/mailbox bandwidth in typical setups.

5. Hardware config pointer lifetime risk removed:
- VCU now deep-copies `vcu_hw_s` during `vcu_init(...)` instead of storing caller pointer directly.

These updates keep the architectural goals of the refactor while directly addressing real-time and safety concerns raised in review.

## Practical Guidance for Team Use

## Keep this boundary stable
If someone asks "Where should this CAN bit mapping go?", the answer should be:
- adapter/schema, not VCU core.

## Keep VCU core semantic
VCU should operate on:
- physical signals
- normalized values
- faults
- semantic commands

## Keep protocol evolution localized
If message IDs, scales, counters, or CRCs change:
- change adapter/schema
- avoid touching core logic unless behavior changes.

## Conclusion
The CAN refactor is not just a code-style change. It is an architectural change that:
- improves determinism,
- supports board consolidation complexity,
- reduces coupling to one CAN driver/schema,
- and makes safety-critical logic easier to test and maintain.

For MKVII VCU, this is the safer and more scalable direction.
