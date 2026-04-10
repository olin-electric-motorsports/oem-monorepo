cc_library(
    name = "mlx90640",
    srcs = [
        "functions/MLX90640_API.c",
    ],
    hdrs = [
        "headers/MLX90640_API.h",
        "headers/MLX90640_I2C_Driver.h",  # 👈 ADD THIS
    ],
    includes = ["headers"],  # 👈 important
    visibility = ["//visibility:public"],
)