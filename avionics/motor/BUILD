package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_test")

py_binary(
    name = "gen_lookup_table",
    srcs = ["gen_lookup_table.py"],
    deps = [
        requirement("numpy"),
        requirement("python-gflags"),
    ],
)

py_test(
    name = "gen_lookup_table_test",
    srcs = ["gen_lookup_table_test.py"],
    deps = [":gen_lookup_table"],
)

py_binary(
    name = "motor_client",
    srcs = ["motor_client.py"],
    data = ["//avionics/motor/firmware:io.c"],
    tags = ["tms570_util"],
    deps = [
        ":gen_lookup_table",
        "//avionics/common:aio",
        "//avionics/common:cmd_client",
        "//avionics/common:h2py_actuator_types",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/common:h2py_safety_codes",
        "//avionics/firmware/params:param_util",
        "//avionics/motor/firmware:h2py_flags",
        "//avionics/network:h2py_aio_labels",
        "//lib/python:c_helpers",
        requirement("scipy"),
    ],
)

py_test(
    name = "motor_client_test",
    srcs = ["motor_client_test.py"],
    flaky = True,
    tags = ["exclusive"],
    deps = [
        ":motor_client",
        "//lib/python:test_util",
    ],
)

py_binary(
    name = "motor_angle_cal",
    srcs = ["motor_angle_cal.py"],
    data = ["//avionics/motor/firmware:io.c"],
    tags = ["tms570_util"],
    deps = [
        ":gen_lookup_table",
        "//avionics/common:aio",
        "//avionics/common:h2py_motor_util",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/firmware/params:common_py",
        "//avionics/motor:motor_client",
        "//avionics/motor/firmware:config_params_py",
        "//avionics/motor/firmware:h2py_params",
        "//avionics/network:h2py_aio_labels",
        "//lib/python:c_helpers",
        "//system:h2py_labels",
        requirement("numpy"),
        requirement("PySide"),
        requirement("python-gflags"),
        requirement("scipy"),
    ],
)

py_binary(
    name = "motor_plotter",
    srcs = ["motor_plotter.py"],
    tags = ["tms570_util"],
    deps = [
        "//avionics/common:aio",
        "//avionics/common:h2py_motor_adc_defines",
        "//avionics/common:h2py_motor_thermal_types",
        "//avionics/motor/firmware:h2py_pack_logging",
        requirement("numpy"),
        requirement("pyqtgraph"),
        requirement("PySide"),
    ],
)

py_test(
    name = "motor_plotter_test",
    srcs = ["motor_plotter_test.py"],
    deps = [":motor_plotter"],
)

py_binary(
    name = "torque_cell_translator",
    srcs = ["torque_cell_translator.py"],
    tags = ["tms570_util"],
    deps = [
        "//avionics/common:aio",
    ],
)
