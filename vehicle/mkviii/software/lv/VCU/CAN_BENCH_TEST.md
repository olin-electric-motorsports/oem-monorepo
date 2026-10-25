# VCU CAN transmit verification

Verified on 2026-10-24 using the reviewed firmware from `30ac2fe`, built at
checkout `6502db0` (which only moved host tests). The user programmed the board
with OpenOCD, received `Verified OK`, and reset the MCU.

ELF SHA256:
`5c669c674ef1e704b45fc2765e2293c466a2aabaf9f3fa17db1c34faa580d543`

## Setup and results

- Isolated low-voltage bench: VCU, PCIe breakout, Arduino 3.3 V/5 V power, and
  CANable. Roadkill, the inverter, motor, and high-voltage system were disconnected.
- The unpowered CAN H/L resistance was approximately 60 Ohm with two terminations.
- Classic CAN, standard identifiers, 500 kbit/s; CANable used normal mode with
  automatic acknowledgments. No application messages were transmitted by the PC.
- The capture recorded 10,802 messages over approximately 27 seconds after
  firmware startup. Each message type arrived at roughly 100 messages/second.

| CAN ID | Message | Bytes | Received |
| --- | --- | ---: | ---: |
| `0x0C0` | Motor command | 8 | 2,701 |
| `0x700` | Throttle state | 8 | 2,701 |
| `0x701` | BSPD state | 7 | 2,700 |
| `0x702` | VCU status | 5 | 2,700 |

Both throttle ADC readings changed together between zero and approximately 4,000
counts about every two seconds, following the Arduino LOW/released-input test
pattern. The heartbeat alternated. Every captured motor command requested zero
torque and inverter disable; ready-to-drive stayed false. Sensor range and
disconnected shutdown-chain faults were present.

The key hardware-setup repair is `6233b4e`: `Src/fdcan.c` selects the 16 MHz HSE
CAN clock, enables FDCAN/GPIOA clocks, and configures PA11 RX and PA12 TX as AF9.
The tested image also includes the transmit priority/error handling in `30ac2fe`.

## Repeat the observation

From the checkout, generate the database and use the standard cantools viewer:

```sh
bazel build --config=m4 //vehicle/mkviii/software/lv/VCU:vcu_can_api
python -m cantools monitor --bus-type slcan --channel /dev/ttyACM0@115200 \
  --bit-rate 500000 bazel-bin/vehicle/mkviii/mkviii.dbc
```

Identify the CANable port before use; its device name can change. Open the viewer
before starting the VCU. Look for an increasing Received count and changing
throttle readings. Press `q` to quit or `p` to pause the display.

This verifies VCU transmission through CANable to the computer and the observed
input transitions. It does not verify received-command handling, ready-to-drive,
real inverter control, sensor calibration, or all protection functions. Host USB
receive timestamps do not establish exact MCU task timing. The complete reviewed
image was tested; the initialization fix was not flashed in isolation.
