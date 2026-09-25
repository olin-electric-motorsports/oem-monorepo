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

cc_library(
    name = "bms_headers",
    hdrs = [
        "bms.h",
        "bms_config.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//common/adc:adc",
        "//common/spi:spi",
        "//vehicle/common/adbms1818:ADBMS181x",
    ],
)

stm32_firmware(
    name = "bms.elf",
    srcs = [
        "bms.c",
        "bms_config.c",
    ],
    deps = [
        ":bms_headers",
        "//vehicle/common/adbms1818:ADBMS1818",
        "//vehicle/mkviii/software/integration/bms/tasks",
        "//vehicle/mkviii/software/integration/bms/utils:bms_utils",
    ],
)

# exports_files([
#     "bms.yml",
# ])
