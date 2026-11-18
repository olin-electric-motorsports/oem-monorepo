# AIR control firmware for the Mark VIII (STM32G441).
#
#   Build:           bazel build --config=m4 //vehicle/mkviii/software/hv/air_control:air.elf
#   Flash:           bazel run --config=m4 //vehicle/mkviii/software/hv/air_control:air_flash
#   Debug:           bazel run -c dbg --config=m4 //vehicle/mkviii/software/hv/air_control:air_debug
#   New chip, once:  bazel run --config=m4 //vehicle/mkviii/software/hv/air_control:air_initialize
#
# The BOOT0 pin isn't connected on the AIR PCB, so a new chip needs
# air_initialize once before it will reliably boot the flashed firmware.

load("//common:macros.bzl", "stm32_firmware")
load("//common/can_api:can_defs.bzl", "can_api_files")


# Generates the CAN code for this board (can_api.c/.h and can_tools.c/.h)
# from air.yml and the car's DBC. air.c includes "can_api.h" from it.
can_api_files(
    name = "air_can_api",
    yaml = "air.yml",
    dbc = "//vehicle/mkviii:mkviii", # Points to DBC target
    deps = [
        "//third_party:stm32cubeg4",
    ],
)


# The firmware itself. The macro adds the shared startup code and linker
# script, and creates air_flash, air_debug and air_initialize.
stm32_firmware(
    name = "air.elf",
    srcs = [
        "air.c",
        "air.h",
    ],
    deps = [
        ":air_can_api",
    ],
)

# Lets //vehicle/mkviii:mkviii read air.yml to build the car's DBC
exports_files([
    "air.yml",
])
