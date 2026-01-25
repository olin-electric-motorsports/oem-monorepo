load("@rules_python//python:defs.bzl", "py_binary", "py_test")
load("@pip//:requirements.bzl", "requirement")

py_binary(
    name = "convert",
    srcs = ["convert.py"],
    deps = [
        requirement("click"),
    ],
    visibility = ["//visibility:public"],
)
