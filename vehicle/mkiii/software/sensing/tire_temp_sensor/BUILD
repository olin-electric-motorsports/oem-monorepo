load("//common:macros.bzl", "stm32_firmware")

stm32_firmware(
    name = "tire_temp.elf",
    srcs = [
        "tire_temp.c",
        "tire_temp.h",
        # If you have interrupts specific to throttle, add them here:
        # "interrupts.c", 
    ],
)