package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_cc_binary",
    "makani_cc_library",
    "makani_cc_test",
)

makani_cc_library(
    name = "logger",
    srcs = ["logger.cc"],
    hdrs = ["logger.h"],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/linux:aio_interface",
        "//avionics/recorder:logger_types",
        "@usr_lib//:glog",
    ],
)

makani_cc_test(
    name = "logger_test",
    srcs = ["logger_test.cc"],
    deps = [
        ":logger",
    ],
)

makani_cc_binary(
    name = "logger_service",
    srcs = ["logger_service.cc"],
    deps = [
        ":logger",
        "//common:macros",
    ],
)

makani_cc_binary(
    name = "log_command",
    srcs = ["log_command.cc"],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/linux:aio_interface",
        "//avionics/recorder:logger_types",
        "//common:macros",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_test(
    name = "log_command_test",
    srcs = ["log_command_test.cc"],
    data = [
        ":log_command",
    ],
)
