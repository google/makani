package(default_visibility = ["//avionics:__subpackages__"])

load(
    "//avionics/firmware/monitors:build_rules.bzl",
    "ina219_genrule",
    "mcp342x_genrule",
    "mcp9800_genrule",
    "si7021_genrule",
)
load("//lib/bazel:c_rules.bzl", "makani_c_library")

# C libraries.

makani_c_library(
    name = "monitors",
    srcs = ["monitors.c"],
    hdrs = ["monitors.h"],
    deps = [
        ":types",
        "//avionics/firmware/monitors:ina219",
        "//common:macros",
    ],
)

makani_c_library(
    name = "types",
    srcs = [
        "motor_ina219_types.c",
        "motor_mcp342x_types.c",
        "motor_mcp9800_types.c",
        "motor_si7021_types.c",
    ],
    hdrs = [
        "motor_ina219_types.h",
        "motor_mcp342x_types.h",
        "motor_mcp9800_types.h",
        "motor_si7021_types.h",
        "types.h",
    ],
    deps = [
        "//avionics/common:faults",
        "//avionics/firmware/monitors:ina219_types",
        "//avionics/firmware/monitors:mcp342x_types",
        "//avionics/firmware/monitors:mcp9800_types",
        "//avionics/firmware/monitors:si7021_types",
        "//avionics/firmware/serial:motor_serial_params_c",
        "//avionics/motor/firmware:config_params_c",
    ],
)

# Code generation rules.

ina219_genrule(
    name = "motor_ina219",
    deps = [
        "//avionics/firmware/drivers:h2py_ina219_types",
        "//avionics/firmware/serial:motor_serial_params_py",
    ],
)

mcp342x_genrule(
    name = "motor_mcp342x",
    deps = [
        "//avionics/firmware/drivers:h2py_mcp342x_types",
        "//avionics/firmware/serial:motor_serial_params_py",
        "//avionics/motor/firmware:config_params_py",
    ],
)

mcp9800_genrule(
    name = "motor_mcp9800",
    deps = [
        "//avionics/firmware/drivers:h2py_mcp9800_types",
        "//avionics/firmware/serial:motor_serial_params_py",
    ],
)

si7021_genrule(
    name = "motor_si7021",
    deps = [
        "//avionics/firmware/drivers:h2py_si7021_types",
        "//avionics/firmware/serial:motor_serial_params_py",
    ],
)
