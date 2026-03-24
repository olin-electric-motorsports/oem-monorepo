#!/bin/bash


# We are writing to the FLASH_OPTR register, refer to data sheet.
# Mask: 0x0C000000 (We only want to change bits 26 and 27)
# Value: 0x04000000 (Sets bit 26 to 1, sets bit 27 to 0)

openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "
init
reset halt


stm32g4x option_read 0 0x20


# Corrected Hex: 0x08000000 sets bit 27 high and bit 26 low.
stm32g4x option_write 0 0x20 0x08000000 0x0C000000


# Corrected Syntax: Only pass the bank ID (0)
stm32g4x option_load 0

reset run
exit
"

