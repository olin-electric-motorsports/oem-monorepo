cc_library(
    name = "hdrs",
    hdrs = [
        "spi.h",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "spi",
    srcs = [
        "spi.c",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":hdrs",
        "//third_party:stm32cubeg4", # Link to the target defined above
    ],
)