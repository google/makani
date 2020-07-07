package(default_visibility = ["//avionics:__subpackages__"])

load("//lib/bazel:c_rules.bzl", "makani_c_library")

# C libraries.

makani_c_library(
    name = "aio_node_status",
    srcs = ["aio_node_status.c"],
    hdrs = ["aio_node_status.h"],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/common:bits",
        "//avionics/firmware/cpu:registers",
    ],
)

makani_c_library(
    name = "slow_status",
    srcs = ["slow_status.c"],
    hdrs = ["slow_status.h"],
    deps = [
        ":aio_node_status",
        "//avionics/common:avionics_messages",
        "//avionics/common:build_info",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/firmware/identity:board_hardware",
        "//avionics/firmware/identity:carrier_hardware",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/serial",
        "//avionics/firmware/startup:ldscript",
        "//avionics/network:message_type",
    ],
)
