def stm32_firmware(name, srcs = [], deps = []):
    """
    Macro to define an STM32G4 firmware binary.
    
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

    flash_target_name = name.replace(".elf", "") + "_flash"
    
    native.sh_binary(
        name = flash_target_name,
        srcs = ["//common:flash.sh"],
        # Automatically pass the location of the compiled ELF to the script
        args = ["$(location :%s)" % name],
        # Tell Bazel that this script needs the ELF file to exist
        data = [":" + name],
    )

    
    debug_target_name = name.replace(".elf", "") + "_debug"
    native.sh_binary(
        name = debug_target_name,
        srcs = ["//common:debug.sh"],
        args = ["$(location :%s)" % name],
        data = [":" + name],
    )