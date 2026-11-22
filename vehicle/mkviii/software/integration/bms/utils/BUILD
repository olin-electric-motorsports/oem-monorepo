cc_library(
    name = "bms_utils",
    srcs = [
        "cell_balancing.c",
        "fault.c",
        "i2c_helpers.c",
    ],
    hdrs = [
        "cell_balancing.h",
        "fault.h",
        "i2c_helpers.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//vehicle/common/adbms1818:ADBMS181x",
        "//vehicle/mkviii/software/integration/bms:bms_can_api",
        "//vehicle/mkviii/software/integration/bms:bms_headers",
    ],
)
