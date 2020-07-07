package(default_visibility = ["//visibility:public"])

load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/python/pack2:build_rules.bzl", "pack2")

# C libraries.

makani_c_library(
    name = "decode",
    srcs = ["decode.c"],
    hdrs = ["decode.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":serial_params_c",
    ],
)

makani_c_library(
    name = "board_serial",
    srcs = ["board_serial.c"],
    hdrs = ["board_serial.h"],
    visibility = ["//avionics:__subpackages__"],
    deps = [
        ":decode",
        "//avionics/firmware/params:flash",
    ],
)

makani_c_library(
    name = "carrier_serial",
    srcs = ["carrier_serial.c"],
    hdrs = ["carrier_serial.h"],
    visibility = ["//avionics:__subpackages__"],
    deps = [
        ":decode",
        "//avionics/firmware/params",
    ],
)

makani_c_library(
    name = "serial",
    visibility = ["//avionics:__subpackages__"],
    deps = [
        ":board_serial",
        ":carrier_serial",
    ],
)

# Pack2 libraries.

pack2(
    name = "aio_serial_params",
    src = "aio_serial_params.p2",
    deps = [":serial_params"],
)

pack2(
    name = "batt_serial_params",
    src = "batt_serial_params.p2",
    deps = [":serial_params"],
)

pack2(
    name = "cs_serial_params",
    src = "cs_serial_params.p2",
    deps = [":serial_params"],
)

pack2(
    name = "fc_serial_params",
    src = "fc_serial_params.p2",
    deps = [":serial_params"],
)

pack2(
    name = "ground_io_serial_params",
    src = "ground_io_serial_params.p2",
    deps = ["//avionics/firmware/serial:serial_params"],
)

pack2(
    name = "joystick_serial_params",
    src = "joystick_serial_params.p2",
    deps = ["//avionics/firmware/serial:serial_params"],
)

pack2(
    name = "loadcell_serial_params",
    src = "loadcell_serial_params.p2",
    deps = ["//avionics/firmware/serial:serial_params"],
)

pack2(
    name = "motor_serial_params",
    src = "motor_serial_params.p2",
    deps = [":serial_params"],
)

pack2(
    name = "mvlv_serial_params",
    src = "mvlv_serial_params.p2",
    deps = [":serial_params"],
)

pack2(
    name = "recorder_serial_params",
    src = "recorder_serial_params.p2",
    deps = ["//avionics/firmware/serial:serial_params"],
)

pack2(
    name = "serial_params",
    src = "serial_params.p2",
)

pack2(
    name = "servo_serial_params",
    src = "servo_serial_params.p2",
    deps = ["//avionics/firmware/serial:serial_params"],
)

pack2(
    name = "short_stack_serial_params",
    src = "short_stack_serial_params.p2",
    deps = [":serial_params"],
)

# Parameter files.
# TODO: These rules should probably go away once we have proper MES
# integration.

filegroup(
    name = "serial_params_templates",
    srcs = glob(["*_serial_params.yaml"]),
)
