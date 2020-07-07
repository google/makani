load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_library")
load("//lib/python/pack2:build_rules.bzl", "packdb")

py_library(
    name = "pack2",
    srcs = [
        "__init__.py",
        "backend.py",
        "backend_c.py",
        "backend_py.py",
        "generator.py",
        "metadata.py",
        "parser.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//lib/python:c_helpers",
        "//lib/python:string_util",
        requirement("ply"),
        requirement("python-gflags"),
    ],
)

py_library(
    name = "py_types",
    srcs = ["py_types.py"],
    visibility = ["//visibility:public"],
)

packdb(
    name = "paramdb",
    packs = [
        "//avionics/batt/firmware:config_params",
        "//avionics/cs/firmware:config_params",
        "//avionics/drum/firmware:config_params",
        "//avionics/fc/firmware:config_params",
        "//avionics/firmware/serial:aio_serial_params",
        "//avionics/firmware/serial:batt_serial_params",
        "//avionics/firmware/serial:cs_serial_params",
        "//avionics/firmware/serial:fc_serial_params",
        "//avionics/firmware/serial:ground_io_serial_params",
        "//avionics/firmware/serial:joystick_serial_params",
        "//avionics/firmware/serial:loadcell_serial_params",
        "//avionics/firmware/serial:motor_serial_params",
        "//avionics/firmware/serial:mvlv_serial_params",
        "//avionics/firmware/serial:recorder_serial_params",
        "//avionics/firmware/serial:serial_params",
        "//avionics/firmware/serial:servo_serial_params",
        "//avionics/firmware/serial:short_stack_serial_params",
        "//avionics/gps/firmware:config_params",
        "//avionics/joystick/firmware:calib_params",
        "//avionics/joystick/firmware:config_params",
        "//avionics/loadcell/firmware:calib_params",
        "//avionics/motor/firmware:calib_params",
        "//avionics/motor/firmware:config_params",
        "//avionics/platform/firmware:config_params",
        "//avionics/plc/firmware:config_params",
        "//avionics/servo/firmware:calib_params",
        "//avionics/servo/firmware:config_params",
    ],
    visibility = ["//avionics/firmware/params:__pkg__"],
)
