cc_library(
    name = "ADBMS1818",
    srcs = [
        "ADBMS1818.c",
    ],
    hdrs = ["ADBMS1818.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":ADBMS181x",
    ],
)

cc_library(
    name = "ADBMS181x",
    srcs = ["ADBMS181x.c"],
    hdrs = [
        "ADBMS181x.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":bms_hardware",
    ],
)

cc_library(
    name = "bms_hardware",
    srcs = [
        "bms_hardware.c",
    ],
    hdrs = [
        "bms_hardware.h",
    ],
    
    visibility = ["//visibility:private"], 
    deps = [
        
        "//common/spi", 
    ],
)