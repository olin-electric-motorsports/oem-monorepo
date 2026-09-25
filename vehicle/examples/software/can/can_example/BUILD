load("//common:macros.bzl", "stm32_firmware")
load("//common/can_api:can_defs.bzl", "can_api_files")


can_api_files(
    name = "can_example_can_api",
    yaml = "can_example.yml",
    dbc = "//vehicle/examples:example", # Points to DBC target
    deps = [
        "//third_party:stm32cubeg4",
    ],
)


stm32_firmware(
    name = "can_example.elf",
    srcs = [
        "can_example.c",
        "can_example.h",
        # If you have interrupts specific to this example, add them here:
    ],
    deps = [
        ":can_example_can_api", 
    ],
)

exports_files([
    "can_example.yml",
])