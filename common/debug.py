"""Cross-platform STM32G4 debug script.

Launches OpenOCD in the background, then connects arm-none-eabi-gdb
with source path mappings for Bazel-built firmware.

Usage: python debug.py <path_to_elf>
"""

import atexit
import os
import platform
import subprocess
import sys
import time
from pathlib import Path


openocd_process = None


def cleanup():
    """Kill the background OpenOCD process on exit."""
    global openocd_process
    if openocd_process is not None:
        try:
            openocd_process.terminate()
            openocd_process.wait(timeout=3)
        except Exception:
            try:
                openocd_process.kill()
            except Exception:
                pass
        openocd_process = None


def kill_existing_openocd():
    """Kill any existing OpenOCD processes."""
    if platform.system() == "Windows":
        subprocess.run(
            ["taskkill", "/F", "/IM", "openocd.exe"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    else:
        subprocess.run(
            ["pkill", "-x", "openocd"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def find_bazel_external_path(repo_root):
    """Locate the Bazel external dependencies path via the bazel-<name> symlink."""
    workspace_name = repo_root.name
    bazel_symlink = repo_root / f"bazel-{workspace_name}"

    if bazel_symlink.exists():
        external_path = repo_root / f"bazel-{workspace_name}" / "external"
        if external_path.exists():
            return str(external_path)

    return None


def compute_source_dir(elf_arg):
    """Derive the local source directory from the Bazel output path."""
    parent = str(Path(elf_arg).parent)
    # Strip Bazel output prefixes
    for prefix in ["bazel-bin/", "bazel-bin\\", "bazel-out/"]:
        if prefix in parent:
            parent = parent.split(prefix, 1)[1]
            # Also strip the config segment for bazel-out (e.g., "fastbuild/bin/")
            if prefix == "bazel-out/":
                parts = parent.split("/", 2)
                if len(parts) > 2:
                    parent = parts[2]
            break
    return parent


def main():
    global openocd_process

    if len(sys.argv) < 2:
        print("Error: No ELF file provided.")
        sys.exit(1)

    elf_file = sys.argv[1]

    # Resolve the absolute path of the ELF
    elf_abs = str(Path(elf_file).resolve())

    # Switch to the correct project root (Bazel provides BUILD_WORKING_DIRECTORY)
    build_working_dir = os.environ.get("BUILD_WORKING_DIRECTORY")
    if build_working_dir:
        os.chdir(build_working_dir)
    repo_root = Path.cwd()

    # Compute the local source directory
    source_dir = compute_source_dir(sys.argv[1])

    # Find Bazel external libs for HAL source mapping
    external_path = find_bazel_external_path(repo_root)

    print("--------------------------------------------------")
    print(f" Debugging: {elf_file}")
    print(f" Repo Root: {repo_root}")
    print("--------------------------------------------------")

    # Kill any existing OpenOCD instances
    kill_existing_openocd()

    # Register cleanup
    atexit.register(cleanup)

    # Start OpenOCD in the background
    openocd_process = subprocess.Popen(
        [
            "openocd",
            "-f", "interface/stlink.cfg",
            "-f", "target/stm32g4x.cfg",
            "-c", "init; reset halt",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(2)

    # Build GDB command
    gdb_cmd = [
        "arm-none-eabi-gdb",
        elf_abs,
        "-ex", "target remote localhost:3333",
        "-ex", "monitor reset halt",
        "-ex", "load",
        "-ex", "dir .",
        "-ex", f"dir {source_dir}",
    ]

    # Add HAL source path mappings if we found the external libs
    if external_path:
        print(f"Found External Libs: {external_path}")
        gdb_cmd += ["-ex", f"set substitute-path external {external_path}"]
        # On Linux, Bazel uses /proc/self/cwd in some paths
        if platform.system() != "Windows":
            gdb_cmd += ["-ex", f"set substitute-path /proc/self/cwd/external {external_path}"]

    gdb_cmd += [
        "-ex", "break main",
        "-ex", "continue",
    ]

    # TUI mode: use it on Unix terminals, skip on Windows CMD/PowerShell
    # (Windows Terminal + mintty may support it, but it's unreliable)
    if platform.system() != "Windows":
        gdb_cmd.insert(1, "-tui")
        gdb_cmd += ["-ex", "layout src"]

    subprocess.run(gdb_cmd)


if __name__ == "__main__":
    main()
