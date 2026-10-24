# Firmware host regression checks

Build the generated VCU CAN files once:

```sh
bazel build --config=m4 //vehicle/mkviii/software/lv/VCU:vcu_can_api
python3 tests/firmware/run_tests.py
```

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
