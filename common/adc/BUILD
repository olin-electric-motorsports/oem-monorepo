cc_library(
    name = "hdrs",
    hdrs = [
        "adc.h",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "adc",
    srcs = [
        "adc.c",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":hdrs",
        "//third_party:stm32cubeg4", # Link to the target defined above
    ],
)