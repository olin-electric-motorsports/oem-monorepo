load("@rules_python//python:defs.bzl", "py_binary")
load("@pip//:requirements.bzl", "requirement")

# # This just defines yaml handler so bazel can see it
# py_library(
#     name = "yaml_handler",
#     srcs = [
#         "utils/yaml_handler.py",
#     ],
#     visibility = ["//visibility:public"],
# )

# generates DBC files from YAML files using cantools
py_binary(
    name = "dbc_generator",
    srcs = [
        "dbc_generator.py",
        "utils/yaml_handler.py",  # Explicitly tell Bazel to include this
        "utils/parse_dbc.py",     
    ],
    imports = ["."],
      deps = [
        requirement("cantools"),
        requirement("PyYAML"),
    ],
    visibility = ["//visibility:public"],
)