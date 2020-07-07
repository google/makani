package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

py_library(
    name = "webmonitor_layout",
    srcs = [
        "autogen.py",
        "base.py",
        "loader.py",
        "memory.py",
    ],
    deps = [
        ":webmonitor_layout_util",
        "//avionics/linux/swig:aio_helper",
        "//gs/monitor2/apps/plugins:webmonitor_plugins",
        "//gs/monitor2/project:webmonitor_settings",
        "//lib/python:ctype_util",
        "//lib/python:debug_util",
        "//lib/python:struct_tree",
        "//lib/python/h5_utils:numpy_utils",
    ],
)

py_library(
    name = "base_indicator",
    srcs = [
        "indicator.py",
    ],
    deps = [
        ":webmonitor_layout_util",
        "//analysis/checks:common",
        "//gs/monitor2/apps/receiver:aio_util",
    ],
)

py_library(
    name = "webmonitor_layout_util",
    srcs = [
        "checks.py",
        "layout_util.py",
        "stoplights.py",
        "widgets.py",
    ],
    deps = [
        "//gs/monitor2/project:webmonitor_settings",
    ],
)

py_test(
    name = "layout_test",
    srcs = ["layout_test.py"],
    deps = [
        ":webmonitor_layout",
        "//gs/monitor2/apps/receiver:receiver_test_util",
        "//lib/python:struct_tree",
    ],
)
