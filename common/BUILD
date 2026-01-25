package(default_visibility = ["//visibility:public"])

# The Configuration Header 
cc_library(
    name = "hal_config",
    hdrs = ["stm32g4xx_hal_conf.h"],
    includes = ["."], # Makes the header visible to the drivers
)

# The Core System 
cc_library(
    name = "stm32g4_core",
    srcs = ["startup.c"],
    deps = [
        "//third_party:stm32cubeg4", # Link to the target defined above
    ],
    alwayslink = True,
)

# The Linker Script
filegroup(
    name = "linker_script",
    srcs = ["stm32g441.ld"],
)

platform(
    name = "stm32_platform",
    constraint_values = [
        "@platforms//cpu:armv7e-m", # Cortex-M4/M7
        "@platforms//os:none",      # Bare metal
    ],
)

exports_files(["flash.sh", "debug.sh"])