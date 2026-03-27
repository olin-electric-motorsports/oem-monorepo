def stm32_firmware(name, srcs = [], deps = []):
    """
    Macro to define an STM32G4 firmware binary.

    Creates a cc_binary target for the ELF. Flash, debug, and initialize
    are handled by invoking the Python scripts in common/ directly — Bazel's
    py_binary cannot resolve a Python toolchain for the bare-metal target
    platform required by --config=m4.

    After building, use the companion scripts:
        python common/flash.py      bazel-bin/<pkg>/<name>
        python common/debug.py      bazel-bin/<pkg>/<name>
        python common/setup_chip.py

    Args:
        name: The name of the output binary (e.g., "throttle.elf")
        srcs: Source files (main.c, .h files, etc.)
        deps: Additional libraries
    """
    native.cc_binary(
        name = name,
        srcs = srcs,
        deps = deps + [
            "//common:stm32g4_core",  # Links startup code & HAL config
        ],
        additional_linker_inputs = ["//common:linker_script"],
        linkopts = [
            "-T $(execpath //common:linker_script)",
            "-Wl,--no-warn-rwx-segments",
            "-Wl,-Map=output.map",
        ],
        copts = [
            "-Wall",
            # "-Werror",  # Optional: Fails build on warnings
        ],
    )
