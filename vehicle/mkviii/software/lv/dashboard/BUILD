load("@rules_python//python:defs.bzl", "py_binary")

# Runs on the dashboard Raspberry Pi, not the laptop. It needs RPi.GPIO,
# python-can and cantools installed on the Pi.
py_binary(
    name = "dashboard",
    srcs = [
        "dashboard.py",
    ],
    main = "dashboard.py",
    visibility = ["//visibility:public"],
)

exports_files([
    "dashboard.yml",
    "mkvi.dbc",
])
