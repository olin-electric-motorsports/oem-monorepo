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
    # target_compatible_with = select({
    #     "//bazel/constraints:atmega16m1": [],
    #     "//bazel/constraints:atmega64m1": [],
    #     "//conditions:default": ["//bazel/constraints:incompatible_mcu"],
    # }),
    visibility = ["//visibility:public"],
    deps = [
        ":hdrs",
        "//third_party:stm32cubeg4", # Link to the target defined above
    ],
)