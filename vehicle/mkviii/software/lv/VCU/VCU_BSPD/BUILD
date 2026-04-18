load("//common:macros.bzl", "stm32_firmware")

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
    ],
)
