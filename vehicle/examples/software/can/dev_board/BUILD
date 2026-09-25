load("//common:macros.bzl", "stm32_firmware")
load("//common/can_api:can_defs.bzl", "can_api_files")


can_api_files(
    name = "dev_board_can_api",
    yaml = "dev_board.yml",
    dbc = "//vehicle/examples:example", # Points to DBC target
    deps = [
        "//third_party:stm32cubeg4",
    ],
)


stm32_firmware(
    name = "dev_board.elf",
    srcs = [
        "dev_board.c",
        "dev_board.h",
        # If you have interrupts specific to dev_board, add them here:
        # "interrupts.c", 
    ],
    deps = [
        ":dev_board_can_api", 
    ],
)

exports_files([
    "dev_board.yml",
])