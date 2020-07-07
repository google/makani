package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

py_library(
    name = "webmonitor_dashboard",
    srcs = glob(["**/*.py"]),
    deps = [
        "//analysis/checks:log_util",
        "//avionics/network:network_config",
        "//gs/monitor2/apps/layout:webmonitor_layout",
        "//gs/monitor2/apps/plugins:webmonitor_plugins",
        "//gs/monitor2/apps/receiver:webmonitor_receiver",
        "//gs/monitor2/project:webmonitor_settings",
        "//lib/bazel:bazel_util",
        "//lib/python:debug_util",
        "//lib/python:json_util",
        "//lib/python:struct_tree",
        "//lib/python/h5_utils",
    ],
)

py_test(
    name = "dashboard_test",
    srcs = ["dashboard_test.py"],
    deps = [
        "//gs/monitor2/apps/layout:webmonitor_layout",
        "//gs/monitor2/apps/plugins:webmonitor_plugins",
        "//gs/monitor2/apps/receiver:receiver_test_util",
        "//lib/python:c_helpers",
        "//lib/python:struct_tree",
        "//lib/python/h5_utils",
        requirement("mock"),
    ],
)
