package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

py_library(
    name = "aio_util",
    srcs = [
        "aio_util.py",
    ],
    deps = [
        "//analysis/checks:check_range",
        "//gs/monitor2/apps/layout:webmonitor_layout_util",
    ],
)

py_library(
    name = "webmonitor_receiver",
    srcs = [
        "receiver_manager.py",
        "urls.py",
        "views.py",
    ],
    deps = [
        ":receivers",
        "//avionics/network:h2py_message_type",
        "//gs/monitor2/project:webmonitor_settings",
    ],
)

py_library(
    name = "receivers",
    srcs = [
        "aio_receiver.py",
        "base_receiver.py",
        "log_receiver.py",
    ],
    deps = [
        "//avionics/linux/swig:aio_helper",
        "//avionics/network:h2py_aio_node",
        "//avionics/network:h2py_message_type",
        "//avionics/network:network_config",
        "//lib/python:struct_tree",
        "//lib/python/trackers:json_obj",
        requirement("Django"),
        requirement("pytz"),
    ],
)

py_library(
    name = "receiver_test_util",
    srcs = [
        "test_util.py",
    ],
    deps = [
        ":receivers",
        "//avionics/network:h2py_aio_node",
        "//avionics/network:network_config",
        "//gs/monitor2/project:webmonitor_settings",
    ],
)

py_test(
    name = "receiver_test",
    srcs = ["receiver_test.py"],
    deps = [
        ":webmonitor_receiver",
        "//avionics/linux/swig:pywrap_aio_util",
        requirement("mock"),
    ],
)
