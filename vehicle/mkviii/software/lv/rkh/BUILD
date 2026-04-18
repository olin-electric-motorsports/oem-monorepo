load("//common:macros.bzl", "stm32_firmware")
load("//common/can_api:can_defs.bzl", "can_api_files")


can_api_files(
    name = "rkh_can_api",
    yaml = "rkh.yml",
    dbc = "//vehicle/mkviii:mkviii",
    deps = [
        "//third_party:stm32cubeg4",
    ],
)

exports_files([
    "rkh.yml",
])

stm32_firmware(
    name = "rkh.elf",
    srcs = [
        "rkh.c",
        "rkh_config.c",
        "rkh.h",
        # If you have interrupts specific to example, add them here:
        # "interrupts.c", 
    ],
    deps = [
        ":rkh_can_api",
    ]
)

