load("//common:macros.bzl", "stm32_firmware")

exports_files([
    "vcu.yml",
])

stm32_firmware(
    name = "vcu.elf",
    srcs = [
        "main.h",
        "vcu.c",
        "vcu.h",
        "vcu_app.c",
        "vcu_config.h",
        "vcu_hal_compat.c",
    ],
)
