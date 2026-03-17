load("@rules_python//python:defs.bzl", "py_binary")
load("@pip//:requirements.bzl", "requirement")

py_binary(
    name = "dbc_generator",
    srcs = [
        "dbc_generator.py",
    ],
      deps = [
        requirement("cantools"),
        requirement("PyYAML"),
    ],
    visibility = ["//visibility:public"],
)