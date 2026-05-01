load("//common:macros.bzl", "stm32_firmware")
load("//common/can_api:can_defs.bzl", "can_api_files")


exports_files([
    "brake_temp.yml",
])


# Can testing
can_api_files(
    name = "brake_temp_can_api",
    yaml = "brake_temp.yml",
    dbc = "//vehicle/mkviii:mkviii.dbc",
    deps = [ 
        "//third_party:stm32cubeg4",
    ],

)

stm32_firmware(
    name = "brake_temp.elf",
    srcs = [
        "brake_temp.c",
        "brake_temp.h",
        "brake_temp_config.c",
        # If you have interrupts specific to throttle, add them here:
        # "interrupts.c", 
        
    ],
    deps = [
        ":brake_temp_can_api",
    ],
)