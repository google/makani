package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

py_library(
    name = "collection",
    srcs = glob(["*_checks.py"]),
    data = [
        "//gs/monitor2/apps/plugins/layouts:gradebook.json",
    ],
    deps = [
        ":network_checks",
        "//analysis/checks:common",
        "//avionics/common:h2py_novatel_types",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/common:h2py_septentrio_types",
        "//avionics/firmware/monitors:h2py_aio_types",
        "//avionics/firmware/monitors:h2py_fc_types",
        "//avionics/firmware/monitors:h2py_servo_types",
        "//avionics/motor/firmware:h2py_flags",
        "//avionics/network:h2py_aio_labels",
        "//avionics/network:h2py_aio_node",
        "//lib/python:c_helpers",
        "//lib/python/batch_sim:scoring_functions",
        requirement("jsmin"),
    ],
)

py_library(
    name = "network_checks",
    srcs = ["network_checks.py"],
    deps = [
        "//analysis/checks:common",
        "//avionics/common:h2py_tether_message_types",
        "//avionics/network:network_config",
    ],
)

py_test(
    name = "tests",
    srcs = ["tests.py"],
    deps = [
        ":collection",
        "//analysis/checks:tests",
        "//avionics/common:h2py_motor_thermal_types",
        "//gs/monitor2/apps/layout:webmonitor_layout_util",
        "//gs/monitor2/apps/receiver:receiver_test_util",
    ],
)
