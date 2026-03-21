load("//common:macros.bzl", "stm32_firmware")
load("//common/can_api:can_defs.bzl", "can_api_files")


can_api_files(
    name = "throttle_can_api",
    yaml = "throttle.yml",
    dbc = "//vehicle/mkviii:mkviii", # Points to your master DBC target
    deps = [
        "//third_party:stm32cubeg4",
    ],
)


stm32_firmware(
    name = "throttle.elf",
    srcs = [
        "throttle.c",
        "throttle.h",
        # If you have interrupts specific to throttle, add them here:
        # "interrupts.c", 
    ],
    deps = [
        ":throttle_can_api", 
    ],
)

exports_files([
    "throttle.yml",
])