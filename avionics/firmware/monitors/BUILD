package(default_visibility = ["//avionics:__subpackages__"])

load("@python_pip//:requirements.bzl", "requirement")
load(
    "//avionics/firmware/monitors:build_rules.bzl",
    "ads7828_genrule",
    "analog_genrule",
    "bq34z100_genrule",
    "ina219_genrule",
    "ltc2309_genrule",
    "ltc4151_genrule",
    "ltc6804_genrule",
    "mcp342x_genrule",
    "mcp9800_genrule",
    "si7021_genrule",
)
load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/bazel:py_rules.bzl", "py_library")

exports_files([
    "generate_ads7828_monitor.py",
    "generate_analog_monitor.py",
    "generate_ina219_monitor.py",
    "generate_ltc2309_monitor.py",
    "generate_ltc4151_monitor.py",
    "generage_ltc6804_monitor.py",
    "generate_mcp342x_monitor.py",
    "generate_mcp9800_monitor.py",
    "generate_si7021_monitor.py",
])

# C libraries.

makani_c_library(
    name = "ads7828",
    srcs = ["ads7828.c"],
    hdrs = ["ads7828.h"],
    deps = [
        ":ads7828_types",
        "//avionics/firmware/drivers:ads7828",
    ],
)

makani_c_library(
    name = "ads7828_types",
    hdrs = ["ads7828_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//avionics/firmware/drivers:ads7828_types",
    ],
)

makani_c_library(
    name = "aio",
    srcs = ["aio.c"],
    hdrs = ["aio.h"],
    deps = [
        ":aio_types",
        ":analog",
        ":ina219",
        ":si7021",
    ],
)

makani_c_library(
    name = "aio_types",
    srcs = [
        "aio_analog_types.c",
        "aio_ina219_types.c",
        "aio_si7021_types.c",
    ],
    hdrs = [
        "aio_analog_types.h",
        "aio_ina219_types.h",
        "aio_si7021_types.h",
        "aio_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":analog_types",
        ":ina219_types",
        ":si7021_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:aio_serial_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "analog",
    srcs = ["analog.c"],
    hdrs = ["analog.h"],
    deps = [
        ":analog_types",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "analog_types",
    hdrs = ["analog_types.h"],
)

makani_c_library(
    name = "batt",
    srcs = ["batt.c"],
    hdrs = ["batt.h"],
    deps = [
        ":analog",
        ":batt_types",
        ":bq34z100",
        ":ltc4151",
        ":ltc6804",
        ":mcp342x",
        ":thermal_types",
    ],
)

makani_c_library(
    name = "batt_types",
    srcs = [
        "batt_analog_types.c",
        "batt_bq34z100_types.c",
        "batt_ltc4151_types.c",
        "batt_ltc6804_types.c",
        "batt_mcp342x_types.c",
    ],
    hdrs = [
        "batt_analog_types.h",
        "batt_bq34z100_types.h",
        "batt_ltc4151_types.h",
        "batt_ltc6804_types.h",
        "batt_mcp342x_types.h",
        "batt_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":analog_types",
        ":bq34z100_types",
        ":ltc4151_types",
        ":ltc6804_types",
        ":mcp342x_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:batt_serial_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "bq34z100",
    srcs = ["bq34z100.c"],
    hdrs = ["bq34z100.h"],
    deps = [
        ":bq34z100_types",
        "//avionics/firmware/drivers:bq34z100",
    ],
)

makani_c_library(
    name = "bq34z100_types",
    hdrs = ["bq34z100_types.h"],
    deps = [
        "//avionics/firmware/drivers:bq34z100_types",
    ],
)

makani_c_library(
    name = "cs",
    srcs = ["cs.c"],
    hdrs = ["cs.h"],
    deps = [
        ":analog",
        ":cs_types",
        ":ina219",
        ":si7021",
    ],
)

makani_c_library(
    name = "cs_types",
    srcs = [
        "cs_analog_types.c",
        "cs_ina219_types.c",
        "cs_si7021_types.c",
    ],
    hdrs = [
        "cs_analog_types.h",
        "cs_ina219_types.h",
        "cs_si7021_types.h",
        "cs_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":analog_types",
        ":ina219_types",
        ":si7021_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:cs_serial_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "fc",
    srcs = ["fc.c"],
    hdrs = ["fc.h"],
    deps = [
        ":analog",
        ":fc_types",
        ":ina219",
        "//avionics/firmware/identity",
    ],
)

makani_c_library(
    name = "fc_types",
    srcs = [
        "fc_analog_types.c",
        "fc_ina219_types.c",
    ],
    hdrs = [
        "fc_analog_types.h",
        "fc_ina219_types.h",
        "fc_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":analog_types",
        ":ina219_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:fc_serial_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "ground_io",
    srcs = ["ground_io.c"],
    hdrs = ["ground_io.h"],
    deps = [
        ":ads7828",
        ":analog",
        ":ground_io_types",
    ],
)

makani_c_library(
    name = "ground_io_types",
    srcs = [
        "ground_io_ads7828_types.c",
        "ground_io_analog_types.c",
    ],
    hdrs = [
        "ground_io_ads7828_types.h",
        "ground_io_analog_types.h",
        "ground_io_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":ads7828_types",
        ":analog_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:ground_io_serial_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "ina219",
    srcs = ["ina219.c"],
    hdrs = ["ina219.h"],
    deps = [
        ":ina219_types",
        "//avionics/firmware/drivers:ina219",
    ],
)

makani_c_library(
    name = "ina219_types",
    hdrs = ["ina219_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//avionics/firmware/drivers:ina219_types",
    ],
)

makani_c_library(
    name = "joystick",
    srcs = ["joystick.c"],
    hdrs = ["joystick.h"],
    deps = [
        ":analog",
        ":ina219",
        ":joystick_types",
    ],
)

makani_c_library(
    name = "joystick_types",
    srcs = [
        "joystick_analog_types.c",
        "joystick_ina219_types.c",
    ],
    hdrs = [
        "joystick_analog_types.h",
        "joystick_ina219_types.h",
        "joystick_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":analog_types",
        ":ina219_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:joystick_serial_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "loadcell",
    srcs = ["loadcell.c"],
    hdrs = ["loadcell.h"],
    deps = [
        ":analog",
        ":loadcell_types",
    ],
)

makani_c_library(
    name = "loadcell_types",
    srcs = [
        "loadcell_analog_types.c",
    ],
    hdrs = [
        "loadcell_analog_types.h",
        "loadcell_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":analog_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:loadcell_serial_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "ltc2309",
    srcs = ["ltc2309.c"],
    hdrs = ["ltc2309.h"],
    deps = [
        ":ltc2309_types",
        "//avionics/firmware/drivers:ltc2309",
    ],
)

makani_c_library(
    name = "ltc2309_types",
    hdrs = ["ltc2309_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//avionics/firmware/drivers:ltc2309_types",
    ],
)

makani_c_library(
    name = "ltc4151",
    srcs = ["ltc4151.c"],
    hdrs = ["ltc4151.h"],
    deps = [
        ":ltc4151_types",
        "//avionics/firmware/drivers:ltc4151",
    ],
)

makani_c_library(
    name = "ltc4151_types",
    hdrs = ["ltc4151_types.h"],
    deps = [
        "//avionics/firmware/drivers:ltc4151_types",
    ],
)

makani_c_library(
    name = "ltc6804",
    srcs = ["ltc6804.c"],
    hdrs = ["ltc6804.h"],
    deps = [
        ":ltc6804_types",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:ltc6804",
    ],
)

makani_c_library(
    name = "ltc6804_types",
    hdrs = ["ltc6804_types.h"],
    deps = [
        "//avionics/firmware/drivers:ltc6804_types",
    ],
)

makani_c_library(
    name = "mcp342x",
    srcs = ["mcp342x.c"],
    hdrs = ["mcp342x.h"],
    deps = [
        ":mcp342x_types",
        "//avionics/firmware/drivers:mcp342x",
    ],
)

makani_c_library(
    name = "mcp342x_types",
    hdrs = ["mcp342x_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//avionics/firmware/drivers:mcp342x_types",
    ],
)

makani_c_library(
    name = "mcp9800",
    srcs = ["mcp9800.c"],
    hdrs = ["mcp9800.h"],
    deps = [
        ":mcp9800_types",
        "//avionics/firmware/drivers:mcp9800",
    ],
)

makani_c_library(
    name = "mcp9800_types",
    hdrs = ["mcp9800_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//avionics/firmware/drivers:mcp9800_types",
    ],
)

makani_c_library(
    name = "mvlv",
    srcs = ["mvlv.c"],
    hdrs = ["mvlv.h"],
    deps = [
        ":analog",
        ":ltc2309",
        ":mcp342x",
        ":mvlv_types",
        ":thermal_types",
    ],
)

makani_c_library(
    name = "mvlv_types",
    srcs = [
        "mvlv_analog_types.c",
        "mvlv_ltc2309_types.c",
        "mvlv_mcp342x_types.c",
    ],
    hdrs = [
        "mvlv_analog_types.h",
        "mvlv_ltc2309_types.h",
        "mvlv_mcp342x_types.h",
        "mvlv_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":analog_types",
        ":ltc2309_types",
        ":mcp342x_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:mvlv_serial_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "recorder",
    srcs = ["recorder.c"],
    hdrs = ["recorder.h"],
    deps = [
        ":analog",
        ":ina219",
        ":recorder_types",
        "//avionics/firmware/identity",
    ],
)

makani_c_library(
    name = "recorder_types",
    srcs = [
        "recorder_analog_types.c",
        "recorder_ina219_types.c",
    ],
    hdrs = [
        "recorder_analog_types.h",
        "recorder_ina219_types.h",
        "recorder_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":analog_types",
        ":ina219_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:recorder_serial_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "servo",
    srcs = ["servo.c"],
    hdrs = ["servo.h"],
    deps = [
        ":analog",
        ":mcp342x",
        ":mcp9800",
        ":servo_types",
        "//avionics/firmware/util:timer",
    ],
)

makani_c_library(
    name = "servo_types",
    srcs = [
        "servo_analog_types.c",
        "servo_mcp342x_types.c",
        "servo_mcp9800_types.c",
    ],
    hdrs = [
        "servo_analog_types.h",
        "servo_mcp342x_types.h",
        "servo_mcp9800_types.h",
        "servo_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":analog_types",
        ":mcp342x_types",
        ":mcp9800_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:servo_serial_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "short_stack",
    srcs = ["short_stack.c"],
    hdrs = ["short_stack.h"],
    deps = [
        ":analog",
        ":mcp342x",
        ":short_stack_types",
    ],
)

makani_c_library(
    name = "short_stack_types",
    srcs = [
        "short_stack_analog_types.c",
        "short_stack_mcp342x_types.c",
    ],
    hdrs = [
        "short_stack_analog_types.h",
        "short_stack_mcp342x_types.h",
        "short_stack_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":analog_types",
        ":mcp342x_types",
        "//avionics/common:faults",
        "//avionics/firmware/serial:short_stack_serial_params_c",
        "//avionics/network:aio_labels",
    ],
)

makani_c_library(
    name = "si7021",
    srcs = ["si7021.c"],
    hdrs = ["si7021.h"],
    deps = [
        ":si7021_types",
        "//avionics/firmware/drivers:si7021",
    ],
)

makani_c_library(
    name = "si7021_types",
    hdrs = ["si7021_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//avionics/firmware/drivers:si7021_types",
    ],
)

makani_c_library(
    name = "thermal_types",
    srcs = [
        "thermal_types.c",
    ],
    hdrs = [
        "thermal_types.h",
    ],
    visibility = ["//visibility:public"],
)

# Python libraries.

py_library(
    name = "generate_monitor_base",
    srcs = [
        "generate_monitor_base.py",
    ],
    visibility = ["//avionics:__subpackages__"],
    deps = [
        "//lib/python:c_helpers",
        requirement("python-gflags"),
    ],
)

# Python/h2py rules.

h2py_library(
    name = "h2py_aio_types",
    function_deps = [
        ":aio_types",
    ],
    header = "aio_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_analog_types",
    function_deps = [
        ":analog_types",
    ],
    header = "analog_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_batt_types",
    function_deps = [
        ":batt_types",
    ],
    header = "batt_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_cs_types",
    function_deps = [
        ":cs_types",
    ],
    header = "cs_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_fc_types",
    function_deps = [
        ":fc_types",
    ],
    header = "fc_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_ina219_types",
    function_deps = [
        ":ina219_types",
    ],
    header = "ina219_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_loadcell_types",
    function_deps = [
        ":loadcell_types",
    ],
    header = "loadcell_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_mcp9800_types",
    function_deps = [
        ":mcp9800_types",
    ],
    header = "mcp9800_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_mvlv_types",
    function_deps = [
        ":mvlv_types",
    ],
    header = "mvlv_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_servo_types",
    function_deps = [
        ":servo_types",
    ],
    header = "servo_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_short_stack_types",
    function_deps = [
        ":short_stack_types",
    ],
    header = "short_stack_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_si7021_types",
    function_deps = [
        ":si7021_types",
    ],
    header = "si7021_types.h",
    visibility = ["//visibility:public"],
)

# Code generation rules.

ads7828_genrule(
    name = "ground_io_ads7828",
    deps = [
        "//avionics/firmware/drivers:h2py_ads7828_types",
        "//avionics/firmware/serial:ground_io_serial_params_py",
    ],
)

analog_genrule(
    name = "aio_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:aio_serial_params_py",
    ],
)

analog_genrule(
    name = "batt_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:batt_serial_params_py",
    ],
)

analog_genrule(
    name = "cs_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:cs_serial_params_py",
    ],
)

analog_genrule(
    name = "fc_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:fc_serial_params_py",
    ],
)

analog_genrule(
    name = "ground_io_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:ground_io_serial_params_py",
    ],
)

analog_genrule(
    name = "joystick_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:joystick_serial_params_py",
    ],
)

analog_genrule(
    name = "loadcell_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:loadcell_serial_params_py",
    ],
)

analog_genrule(
    name = "mvlv_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:mvlv_serial_params_py",
    ],
)

analog_genrule(
    name = "recorder_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:recorder_serial_params_py",
    ],
)

analog_genrule(
    name = "servo_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:servo_serial_params_py",
    ],
)

analog_genrule(
    name = "short_stack_analog",
    deps = [
        "//avionics/firmware/monitors:h2py_analog_types",
        "//avionics/firmware/serial:short_stack_serial_params_py",
    ],
)

ina219_genrule(
    name = "aio_ina219",
    deps = [
        "//avionics/firmware/drivers:h2py_ina219_types",
        "//avionics/firmware/serial:aio_serial_params_py",
    ],
)

ina219_genrule(
    name = "cs_ina219",
    deps = [
        "//avionics/firmware/drivers:h2py_ina219_types",
        "//avionics/firmware/serial:cs_serial_params_py",
    ],
)

ina219_genrule(
    name = "fc_ina219",
    deps = [
        "//avionics/firmware/drivers:h2py_ina219_types",
        "//avionics/firmware/serial:fc_serial_params_py",
    ],
)

ina219_genrule(
    name = "joystick_ina219",
    deps = [
        "//avionics/firmware/drivers:h2py_ina219_types",
        "//avionics/firmware/serial:joystick_serial_params_py",
    ],
)

ina219_genrule(
    name = "recorder_ina219",
    deps = [
        "//avionics/firmware/drivers:h2py_ina219_types",
        "//avionics/firmware/serial:recorder_serial_params_py",
    ],
)

bq34z100_genrule(
    name = "batt_bq34z100",
    deps = [
        "//avionics/firmware/drivers:h2py_bq34z100_types",
        "//avionics/firmware/serial:batt_serial_params_py",
    ],
)

ltc2309_genrule(
    name = "mvlv_ltc2309",
    deps = [
        "//avionics/firmware/drivers:h2py_ltc2309_types",
        "//avionics/firmware/serial:mvlv_serial_params_py",
    ],
)

ltc4151_genrule(
    name = "batt_ltc4151",
    deps = [
        "//avionics/firmware/drivers:h2py_ltc4151_types",
        "//avionics/firmware/serial:batt_serial_params_py",
    ],
)

ltc6804_genrule(
    name = "batt_ltc6804",
    deps = [
        "//avionics/firmware/drivers:h2py_ltc6804_types",
        "//avionics/firmware/serial:batt_serial_params_py",
    ],
)

mcp342x_genrule(
    name = "batt_mcp342x",
    deps = [
        "//avionics/firmware/drivers:h2py_mcp342x_types",
        "//avionics/firmware/serial:batt_serial_params_py",
    ],
)

mcp342x_genrule(
    name = "mvlv_mcp342x",
    deps = [
        "//avionics/firmware/drivers:h2py_mcp342x_types",
        "//avionics/firmware/serial:mvlv_serial_params_py",
    ],
)

mcp342x_genrule(
    name = "servo_mcp342x",
    deps = [
        "//avionics/firmware/drivers:h2py_mcp342x_types",
        "//avionics/firmware/serial:servo_serial_params_py",
    ],
)

mcp342x_genrule(
    name = "short_stack_mcp342x",
    deps = [
        "//avionics/firmware/drivers:h2py_mcp342x_types",
        "//avionics/firmware/serial:short_stack_serial_params_py",
    ],
)

mcp9800_genrule(
    name = "servo_mcp9800",
    deps = [
        "//avionics/firmware/drivers:h2py_mcp9800_types",
        "//avionics/firmware/serial:servo_serial_params_py",
    ],
)

si7021_genrule(
    name = "aio_si7021",
    deps = [
        "//avionics/firmware/drivers:h2py_si7021_types",
        "//avionics/firmware/serial:aio_serial_params_py",
    ],
)

si7021_genrule(
    name = "cs_si7021",
    deps = [
        "//avionics/firmware/drivers:h2py_si7021_types",
        "//avionics/firmware/serial:cs_serial_params_py",
    ],
)
