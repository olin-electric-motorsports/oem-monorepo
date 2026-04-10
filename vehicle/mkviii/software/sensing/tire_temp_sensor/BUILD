load("//common:macros.bzl", "stm32_firmware")

stm32_firmware(
    name = "tire_temp.elf",
    srcs = [
        "tire_temp.c",
        "tire_temp.h",
        "tire_temp_config.c"
        # If you have interrupts specific to throttle, add them here:
        # "interrupts.c", 
    ],
    deps = ["//vehicle/mkviii/software/sensing/tire_temp_sensor/mlx90640-library-master:mlx90640"],
)