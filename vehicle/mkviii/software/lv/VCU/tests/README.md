# VCU host regression checks

Build the generated VCU CAN files once:

```sh
bazel build --config=m4 //vehicle/mkviii/software/lv/VCU:vcu_can_api
python3 vehicle/mkviii/software/lv/VCU/tests/run_tests.py
```

All files in this directory are VCU-specific test support. The headers under
`include/` are local substitutes, not ST official headers. Only this test runner
adds them to its include path; the ARM firmware build uses the official HAL.

A C compiler (`gcc`) and Python 3 are required. There are no additional Python
packages. Pass case names to run selected checks. Every case runs in a separate
process so peripheral initialization state does not leak between cases.

The runner compiles the real VCU, GPIO, shared ADC and generated CAN C sources
with a small HAL substitute and UndefinedBehaviorSanitizer. The substitute can
inject ADC initialization/conversion errors and a limited CAN transmit capacity.
It never connects to USB devices, flashes firmware, or simulates analog accuracy,
CAN electrical timing, or the BSPD circuit. ARM builds and bench checks remain
separate validation steps.

The calibration and APPS latch checks protect the existing temporary calibration
and the existing manual restart policy. The startup check verifies that missing
dashboard input does not accidentally enable the inverter.

CAN checks cover command priority, deferred telemetry, full queues, controller
errors, and submitting an updated Disable command after a fault. A command still
waiting for a hardware slot after the existing 10 ms publication period is a
communication fault. Successful submission means the controller accepted the
message; it does not prove that the inverter received it. After a communication
fault, recovering submissions does not restore ready-to-drive automatically.
Bus-off stays disabled until the controller is reinitialized (for example by an
MCU restart); automatic bus-off restart is deliberately not implemented here.
