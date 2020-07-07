package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_library")

py_library(
    name = "autocheck_gradebook",
    srcs = [],
    data = [
        "gradebook.json",
    ],
)

py_library(
    name = "layouts",
    srcs = glob(["**/*.py"]),
    deps = [
        ":autocheck_gradebook",
        "//analysis/checks:common",
        "//analysis/checks/collection",
        "//avionics/common:gps_receiver_py",
        "//avionics/common:h2py_motor_thermal_types",
        "//avionics/motor/firmware:h2py_flags",
        "//avionics/network:h2py_aio_labels",
        "//avionics/network:network_bandwidth",
        "//avionics/network:network_config",
        "//avionics/network:network_util",
        "//gs/monitor:h2py_monitor_params",
        "//gs/monitor2/apps/layout:base_indicator",
        "//gs/monitor2/apps/layout:webmonitor_layout_util",
        "//gs/monitor2/apps/plugins/indicators",
        "//gs/monitor2/project:webmonitor_settings",
        "//system:h2py_labels",
    ],
)
