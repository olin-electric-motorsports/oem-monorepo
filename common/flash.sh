#!/bin/bash
# Generic STM32G4 Flasher Script
# Usage: ./flash.sh <path_to_elf>

ELF_FILE="$1"

if [ -z "$ELF_FILE" ]; then
    echo "Error: No ELF file provided."
    exit 1
fi

echo "=========================================="
echo " Flashing: $ELF_FILE"
echo "=========================================="

# Run OpenOCD
# Note: $(pwd) is needed because Bazel runs in a sandbox
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
    -c "program $(pwd)/$ELF_FILE verify reset exit"