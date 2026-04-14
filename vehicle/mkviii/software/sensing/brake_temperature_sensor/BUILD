load("//common:macros.bzl", "stm32_firmware")

stm32_firmware(
    name = "brake_temp.elf",
    srcs = [
        "brake_temp.c",
        "brake_temp.h",
        "brake_temp_config.c"
        # If you have interrupts specific to throttle, add them here:
        # "interrupts.c", 
    ],
)