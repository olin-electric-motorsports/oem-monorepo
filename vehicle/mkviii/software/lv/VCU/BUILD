load("//common:macros.bzl", "stm32_firmware")
load("//common/can_api:can_defs.bzl", "can_api_files")

can_api_files(
    name = "vcu_can_api",
    yaml = "vcu.yml",
    dbc = "//vehicle/mkviii:mkviii",
    deps = [
        "//third_party:stm32cubeg4",
    ],
)

exports_files([
    "vcu.yml",
])

cc_library(
    name = "vcu_headers",
    hdrs = glob(["Inc/*.h"]) + ["vcu_config.h"],
    includes = ["Inc", "."],
    deps = [
        "//common/adc:adc",
    ],
    visibility = ["//visibility:private"],
)

stm32_firmware(
    name = "vcu.elf",
    srcs = glob(["Src/*.c"]),
    deps = [
        ":vcu_headers",
        ":vcu_can_api",
    ],
)
