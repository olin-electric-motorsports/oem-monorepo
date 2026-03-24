load("//common/can_api:can_defs.bzl", "dbc_gen")

# Generates main DBC
dbc_gen(
    name = "example",
    srcs = [
    	#"//projects/can_print:can_print.yml",
    	
        # the following are all the DBC files that our car uses.
        "//vehicle/common/gmeter:gmeter.dbc",
        "//vehicle/common/lpms-cu2:lpms-cu2.dbc",

        "//vehicle/common/gps:gps.dbc",
        "//vehicle/common/motor_controller:cm200dx.dbc", # this one depends one which of the two motor controllers we using
        
        # the following are all the YAMl files that define our cars CAN messages. 
        # If a YAML file is not added here, the files cannot refer to those YAML scripts
        "//vehicle/examples/can/can_example:can_example.yml",
        "//vehicle/examples/can/dev_board:dev_board.yml",
    ],
)
