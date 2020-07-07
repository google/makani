package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

py_library(
    name = "base",
    srcs = ["base.py"],
)

py_test(
    name = "network_test",
    size = "small",
    srcs = ["network_test.py"],
    deps = [
        ":base",
        "//avionics/linux/swig:aio_helper",
        "//gs/monitor2/apps/plugins/indicators",
    ],
)

py_test(
    name = "flight_mode_indicator_test",
    size = "small",
    srcs = ["flight_mode_indicator_test.py"],
    deps = [
        ":base",
        "//gs/monitor2/apps/plugins/indicators",
        "//gs/monitor2/apps/receiver:receiver_test_util",
    ],
)

py_test(
    name = "control_test",
    srcs = ["control_test.py"],
    deps = [
        "//avionics/common:aio",
        "//avionics/common:h2py_cvt",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/linux/swig:pywrap_aio_util",
        "//avionics/network:h2py_aio_node",
        "//avionics/network:h2py_message_type",
        "//gs/monitor2/apps/plugins:common",
        "//gs/monitor2/apps/plugins/indicators",
        "//gs/monitor2/apps/receiver:receiver_test_util",
        "//gs/monitor2/high_frequency_filters:h2py_filter_handlers",
        "//lib/python:struct_tree",
        requirement("mock"),
    ],
)

py_test(
    name = "estimator_indicators_test",
    srcs = ["estimator_indicators_test.py"],
    # TODO(b/30958135): Fix flakiness.
    flaky = True,
    deps = [
        "//avionics/linux/swig:pywrap_aio_util",
        "//gs/monitor2/apps/plugins:common",
        "//gs/monitor2/apps/plugins/indicators",
        "//gs/monitor2/apps/receiver:receiver_test_util",
        "//lib/python:struct_tree",
        requirement("mock"),
    ],
)

py_test(
    name = "gps_test",
    srcs = ["gps_test.py"],
    deps = [
        "//gs/monitor2/apps/plugins/indicators",
        "//gs/monitor2/apps/receiver:receiver_test_util",
        "//lib/python:struct_tree",
    ],
)

py_test(
    name = "gs_test",
    srcs = ["gs_test.py"],
    deps = [
        "//avionics/linux/swig:aio_helper",
        "//avionics/linux/swig:pywrap_aio_util",
        "//gs/monitor2/apps/plugins/indicators",
        "//gs/monitor2/apps/receiver:receiver_test_util",
        "//lib/python:struct_tree",
    ],
)

py_test(
    name = "servo_test",
    srcs = ["servo_test.py"],
    deps = [
        "//gs/monitor2/apps/plugins/indicators",
        "//gs/monitor2/apps/receiver:receiver_test_util",
        "//lib/python:struct_tree",
        requirement("mock"),
    ],
)
