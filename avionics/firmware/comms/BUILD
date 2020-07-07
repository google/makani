package(default_visibility = ["//avionics:__subpackages__"])

load("//lib/bazel:c_rules.bzl", "makani_c_library")

# Test functions.

makani_c_library(
    name = "tether_message_test",
    srcs = ["tether_message_test.c"],
    hdrs = ["tether_message_test.h"],
    deps = [
        "//avionics/common:cvt",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:tether_convert",
        "//avionics/common:tether_message",
        "//avionics/firmware/test",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_tms570s",
        "//common:macros",
    ],
)
