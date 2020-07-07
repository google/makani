package(default_visibility = ["//visibility:public"])

load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
)

makani_c_library(
    name = "common",
    srcs = ["common.c"],
    hdrs = ["common.h"],
    deps = [
        ":filter_handlers",
        ":filter_types",
        "//common:macros",
    ],
)

makani_c_library(
    name = "filter_handlers",
    srcs = ["filter_handlers.c"],
    hdrs = ["filter_handlers.h"],
    deps = [
        ":filter_types",
        "//avionics/common:avionics_messages",
        "//avionics/common:cvt",
        "//avionics/common:pack_avionics_messages",
        "//avionics/common:tether_message",
        "//avionics/linux:clock",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:cvt_control_telemetry",
        "//control:system_params",
        "//control/crosswind:crosswind_types",
    ],
)

makani_c_library(
    name = "filter_types",
    hdrs = ["filter_types.h"],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/common:tether_message_types",
        "//avionics/network:aio_node",
        "//common/c_math:common",
    ],
)

h2py_library(
    name = "h2py_filter_types",
    header = "filter_types.h",
    header_deps = [
        ":filter_types",
    ],
)

h2py_library(
    name = "h2py_filter_handlers",
    header = "filter_handlers.h",
    header_deps = [
        ":filter_types",
    ],
)
