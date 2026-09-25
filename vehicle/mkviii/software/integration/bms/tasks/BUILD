cc_library(
    name = "tasks",
    srcs = [
        "current.c",
        "openwire.c",
        "tempurature.c",
        "voltage.c",
    ],
    hdrs = ["tasks.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//vehicle/common/adbms1818:ADBMS181x",
        "//vehicle/mkviii/software/integration/bms:bms_headers",
        "//vehicle/mkviii/software/integration/bms/utils:bms_utils",
    ],
)
