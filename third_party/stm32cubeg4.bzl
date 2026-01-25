def hal_library(name, device, hal_config_lib, hse_value = 25000000, **kwargs):
    """ Creates HAL library specific to project.

    Args:
      name: Name used as prefix for include and source targets.
      device: Stm32 device string used for selecting the correct HAL headers.
      hal_config_lib: Library which exposes "stm32f7xx_hal_conf.h" and can be imported without
          prefix (e.g. #include "stm32f7xx_hal_conf.h") from external libraries.
      hse_value: Value of board HSE clock frequency.
    """
    native.cc_library(
        name = name,
        srcs = [
            "@stm32cubeg4//:hal_source",
            "//third_party:system_core_clock.c",
        ],
        deps = [
            hal_config_lib,
            "@stm32cubeg4//:hal_header_lib",
            "@stm32cubeg4//:cmsis_device",
            "@stm32cubeg4//:cmsis_core",
        ],
        defines = [
            device,
            "HSE_VALUE={}".format(hse_value),
        ],
        **kwargs
    )
