load("//common/can_api:can_defs.bzl", "dbc_gen")

package(default_visibility = ["//visibility:public"])

# Host-only tests: `bazel build --config=m4 //vehicle/...` skips them.
HOST_ONLY = select({
    "@platforms//os:none": ["@platforms//:incompatible"],
    "//conditions:default": [],
})

cc_library(
    name = "air_logic",
    srcs = ["air.c", "air_protocol.c"],
    hdrs = ["air.h", "air_config.h", "air_protocol.h"],
)

cc_binary(
    name = "air.elf",
    srcs = ["air_board.c", "air_board.h", "air_board_config.c", "air_board_config.h", "air_stm32.c"],
    deps = [":air_logic", "//common:stm32g4_core", "//third_party:stm32cubeg4"],
    additional_linker_inputs = ["air.ld"],
    linkopts = ["-T $(location air.ld)", "-Wl,--gc-sections", "-Wl,--no-warn-rwx-segments"],
)

sh_binary(
    name = "air_flash",
    srcs = ["//common:flash.sh"],
    args = ["$(location :air.elf)"],
    data = [":air.elf"],
)

sh_binary(
    name = "air_debug",
    srcs = ["//common:debug.sh"],
    args = ["$(location :air.elf)"],
    data = [":air.elf"],
)

sh_binary(name = "air_initialize", srcs = ["//common:setup_chip.sh"])

cc_test(
    name = "air_test",
    srcs = ["tests/air_test.c"],
    target_compatible_with = HOST_ONLY,
    deps = [":air_logic"],
)

cc_test(
    name = "air_paths_test",
    srcs = ["tests/air_harness.h", "tests/air_paths_test.c"],
    target_compatible_with = HOST_ONLY,
    deps = [":air_logic"],
)

cc_test(
    name = "air_known_issues_test",
    srcs = ["tests/air_harness.h", "tests/air_known_issues_test.c"],
    target_compatible_with = HOST_ONLY,
    deps = [":air_logic"],
)

cc_test(
    name = "air_invariant_test",
    srcs = ["tests/air_invariant_test.c"],
    target_compatible_with = HOST_ONLY,
    deps = [":air_logic"],
)

test_suite(
    name = "host_tests",
    tests = [":air_test", ":air_paths_test", ":air_known_issues_test", ":air_invariant_test"],
)

# A local compatibility DBC until the new BMS interface is finalized.
dbc_gen(
    name = "air_dbc",
    srcs = ["air.yml", "legacy_bms.yml", "//vehicle/common/gmeter:gmeter.dbc"],
)
exports_files(["air.yml"])
