"""Cross-platform STM32G4 chip initialization script.

Configures the FLASH_OPTR register boot pin settings via OpenOCD.
This only needs to be run once on a fresh/erased chip.
"""

import subprocess
import sys


def main():
    # We are writing to the FLASH_OPTR register, refer to data sheet.
    # Mask: 0x0C000000 (We only want to change bits 26 and 27)
    # Value: 0x08000000 (Sets bit 27 high and bit 26 low)
    openocd_commands = """
init
reset halt

stm32g4x option_read 0 0x20

stm32g4x option_write 0 0x20 0x08000000 0x0C000000

stm32g4x option_load 0

reset run
exit
"""

    subprocess.run(
        [
            "openocd",
            "-f", "interface/stlink.cfg",
            "-f", "target/stm32g4x.cfg",
            "-c", openocd_commands.strip(),
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
