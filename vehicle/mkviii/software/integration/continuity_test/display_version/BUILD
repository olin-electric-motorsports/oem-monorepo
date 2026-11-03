load("//common/can_api:can_defs.bzl", "can_api_files")
load("//bazel:defs.bzl", "cc_firmware")

exports_files([
    "cont_test.yml",
])

# Defines the CAN API library
can_api_files(
    name = "cont_test_can_api",
    yaml = "cont_test.yml",
    dbc = "//vehicle/mkviii:mkviii.dbc", # Points to DBC target
    deps = [
        "//third_party:stm32cubeg4",
    ],
)
