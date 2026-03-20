load("@rules_python//python:defs.bzl", "py_binary")
load("@pip//:requirements.bzl", "requirement")
load("@rules_python//python/entry_points:py_console_script_binary.bzl", "py_console_script_binary")

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
        "utils/yaml_handler.py",  # tell Bazel to include this
        "utils/parse_dbc.py",     
    ],
    imports = ["."],
      deps = [
        requirement("cantools"),
        requirement("PyYAML"),
    ],
    visibility = ["//visibility:public"],
)

py_binary(
    name = "c_generator",
    srcs = [
        "c_generator.py",
        "utils/parse_dbc.py",
    ],
    imports = ["."], 
    deps = [
        requirement("cantools"),
        requirement("PyYAML"),
        requirement("Jinja2"),
    ],
    visibility = ["//visibility:public"],
)

# This grabs the CLI executable so we can use it elsewhere
py_console_script_binary(
    name = "cantools_cli",
    pkg = "@pip//cantools", # Points to the installed pip package
    script = "cantools",    # specific console script to run
    visibility = ["//visibility:public"],
)