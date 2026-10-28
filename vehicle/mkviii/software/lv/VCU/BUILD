load("//common:macros.bzl", "stm32_firmware")
load("//common/can_api:can_defs.bzl", "can_api_files", "dbc_gen")

# Extend the vehicle database with the dashboard's legacy message definition.
dbc_gen(
    name = "vcu_can",
    srcs = [
        "//vehicle/mkviii:mkviii",
        "//vehicle/mkviii/software/lv/dashboard:dashboard.yml",
    ],
)

can_api_files(
    name = "vcu_can_api",
    yaml = "vcu.yml",
    dbc = ":vcu_can",
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
