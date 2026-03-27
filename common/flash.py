"""Cross-platform STM32G4 Flasher Script.

Usage: python flash.py <path_to_elf>
"""

import os
import subprocess
import sys
from pathlib import Path


def main():
    if len(sys.argv) < 2:
        print("Error: No ELF file provided.")
        sys.exit(1)

    elf_file = sys.argv[1]

    print("==========================================")
    print(f" Flashing: {elf_file}")
    print("==========================================")

    # Bazel runs in a sandbox — resolve the ELF path relative to cwd.
    # Use forward slashes (POSIX style) since OpenOCD expects them on all platforms.
    elf_path = Path(os.getcwd(), elf_file).as_posix()

    subprocess.run(
        [
            "openocd",
            "-f", "interface/stlink.cfg",
            "-f", "target/stm32g4x.cfg",
            "-c", f"program {elf_path} verify reset exit",
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
