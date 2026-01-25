package(default_visibility = ["//visibility:public"])

filegroup(
    name = "hal_source",
    srcs = glob([
        "Drivers/STM32G4xx_HAL_Driver/Src/*.c",
    ]),
)

cc_library(
    name = "hal_header_lib",
    hdrs = glob([
        "Drivers/STM32G4xx_HAL_Driver/Inc/*.h",
        "Drivers/STM32G4xx_HAL_Driver/Inc/Legacy/*.h",
    ]),
    includes = ["Drivers/STM32G4xx_HAL_Driver/Inc"],
)

cc_library(
    name = "cmsis_device",
    hdrs = glob([
        "Drivers/CMSIS/Device/ST/STM32G4xx/Include/*.h",
    ]),
    includes = ["Drivers/CMSIS/Device/ST/STM32G4xx/Include"],
)

cc_library(
    name = "cmsis_core",
    hdrs = glob([
        "Drivers/CMSIS/Include/*.h",
    ]),
    includes = ["Drivers/CMSIS/Include"],
)





# filegroup(
#     name = "hal_source",
#     srcs = glob(
#         ["Drivers/STM32G4xx_HAL_Driver/Src/**/*.c"],
#         ["Drivers/STM32G4xx_HAL_Driver/Src/**/*template.c"],
#         allow_empty = False,
#     ),
#     visibility = ["//visibility:public"],
# )

# cc_library(
#     name = "hal_header_lib",
#     hdrs = glob(
#         ["Drivers/STM32G4xx_HAL_Driver/Inc/**/*.h"],
#         ["Drivers/STM32G4xx_HAL_Driver/Inc/**/*template.h"],
#         allow_empty = False,
#     ),
#     includes = ["Drivers/STM32G4xx_HAL_Driver/Inc"],
#     visibility = ["//visibility:public"],
# )

# cc_library(
#     name = "cmsis_device",
#     hdrs = glob(
#         ["Drivers/CMSIS/Device/ST/STM32G4xx/Include/*.h"],
#         allow_empty = False,
#     ),
#     includes = ["Drivers/CMSIS/Device/ST/STM32G4xx/Include"],
#     visibility = ["//visibility:public"],
# )

# cc_library(
#     name = "cmsis_core",
#     hdrs = glob(
#         ["Drivers/CMSIS/Core/Include/*.h"],
#         allow_empty = False,
#     ),
#     includes = ["Drivers/CMSIS/Core/Include"],
#     visibility = ["//visibility:public"],
# )
