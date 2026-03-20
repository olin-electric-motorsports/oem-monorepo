
def dbc_gen(name, srcs):
    native.genrule(
        name = name,
        srcs = srcs,
        outs = [
            name,
        ],
        tools = [
            "//common/can_api:dbc_generator",
        ],
        cmd = "$(location //common/can_api:dbc_generator) -o $(OUTS) $(SRCS)",
        visibility = ["//visibility:public"],
    )

def can_api_files(name, yaml, dbc, deps = []):
    
    # 1. Generate the base C files using standard cantools
    native.genrule(
        name = name + "_cantools_gen",
        srcs = [dbc],
        outs = [
            "can_tools.c",
            "can_tools.h",
        ],
        tools = [
            "//common/can_api:cantools_cli",
        ],
        # $(@D) ensures the files drop in the exact output folder Bazel expects
        cmd = "$(location //common/can_api:cantools_cli) generate_c_source --database-name can_tools --output-directory $(@D) $(location " + dbc + ")",
    )

    # Generate OEM wrapper files using Jinja templates
    native.genrule(
        name = name + "_oem_gen",
        srcs = [
            yaml,
            dbc,
            "//common/can_api/templates:c_file.j2",
            "//common/can_api/templates:h_file.j2",
        ],
        outs = [
            "can_api.c",
            "can_api.h",
        ],
        tools = [
            "//common/can_api:c_generator",
        ],
        cmd = """
        $(location //common/can_api:c_generator) \
            --yaml $(location {yaml}) \
            --dbc $(location {dbc}) \
            --c-file $(location //common/can_api/templates:c_file.j2) \
            --h-file $(location //common/can_api/templates:h_file.j2) \
            --output_dir $(@D)
        """.format(yaml=yaml, dbc=dbc),
    )

    # Combine 4 generated files into a single usable C library
    native.cc_library(
        name = name,
        srcs = [
            "can_api.c",
            "can_tools.c",
        ],
        hdrs = [
            "can_api.h",
            "can_tools.h",
        ],
        deps = deps,
        visibility = ["//visibility:public"],
    )
