package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stm32cubeg4",

    target_compatible_with = ["@platforms//os:none"],
    # Compile the external HAL sources + local system clock
    srcs = glob(["system_core_clock.c"]) + [
        "@stm32cubeg4//:hal_source",
    ],
    # Link the headers and config
    deps = [
        "//common:hal_config",          # Points to common/stm32g4xx_hal_conf.h
        "@stm32cubeg4//:hal_header_lib",
        "@stm32cubeg4//:cmsis_device",
        "@stm32cubeg4//:cmsis_core",
    ],
    defines = [
        "STM32G441xx",      # Tells CMSIS which struct definitions to use
        "USE_HAL_DRIVER",   # Enables the HAL headers
    ],
)