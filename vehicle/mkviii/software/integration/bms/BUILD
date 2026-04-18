load("//common:macros.bzl", "stm32_firmware")
load("//common/can_api:can_defs.bzl", "can_api_files")


# can_api_files(
#     name = "bms_can_api",
#     yaml = "bms.yml",
#     dbc = "//vehicle/mkviii/integration/bms:bms", # Points to DBC target
#     deps = [
#         "//third_party:stm32cubeg4",
#     ],
# )

stm32_firmware(
    name = "bms.elf",
    srcs = [
        "bms.c",
        "bms_config.c",
        "bms.h",
        "bms_config.h",
        # If you have interrupts specific to example, add them here:
        # "interrupts.c", 
    ],
    # deps = [
    #     "//vehicle/common/ltc6811:ltc6811",
        
    # ],
)

# exports_files([
#     "bms.yml",
# ])