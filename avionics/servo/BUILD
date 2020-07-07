package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_test")

py_binary(
    name = "servo_client",
    srcs = ["servo_client.py"],
    tags = ["tms570_util"],
    deps = [
        "//avionics/common:aio",
        "//avionics/common:cmd_client",
        "//avionics/common:h2py_actuator_types",
        "//avionics/common:h2py_faults",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/common:h2py_safety_codes",
        "//avionics/common:h2py_servo_types",
        "//avionics/network:h2py_aio_labels",
        "//avionics/network:h2py_aio_node",
        "//avionics/network:h2py_message_type",
        "//avionics/servo/firmware:h2py_r22_param",
        "//lib/python:c_helpers",
    ],
)

py_test(
    name = "servo_client_test",
    srcs = ["servo_client_test.py"],
    flaky = True,
    tags = ["exclusive"],
    deps = [
        ":servo_client",
        "//lib/python:test_util",
    ],
)

py_binary(
    name = "servo_plotter",
    srcs = ["servo_plotter.py"],
    tags = ["tms570_util"],
    deps = [
        "//avionics/common:aio",
        "//avionics/common:h2py_actuator_types",
        "//avionics/common:h2py_faults",
        "//avionics/common:h2py_pack_aio_header",
        "//avionics/common:h2py_servo_types",
        "//avionics/firmware/monitors:h2py_servo_types",
        "//avionics/network:h2py_aio_node",
        "//avionics/network:h2py_message_type",
        "//avionics/servo/firmware:h2py_r22_types",
        "//lib/python:c_helpers",
        "//lib/python:ctype_util",
        requirement("numpy"),
        requirement("pyqtgraph"),
        requirement("PySide"),
    ],
)

py_test(
    name = "servo_plotter_test",
    srcs = ["servo_plotter_test.py"],
    deps = [":servo_plotter"],
)
