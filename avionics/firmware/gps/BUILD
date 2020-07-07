package(default_visibility = ["//avionics:__subpackages__"])

load("//lib/bazel:c_rules.bzl", "makani_c_library")

makani_c_library(
    name = "gps",
    srcs = [
        "gps_interface.c",
        "novatel_config.c",
        "septentrio_config.c",
    ],
    hdrs = [
        "gps_interface.h",
        "novatel_config.h",
        "septentrio_config.h",
    ],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/common:gps_receiver_c",
        "//avionics/common:novatel",
        "//avionics/common:septentrio",
        "//avionics/common:serial_parse",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:novatel",
        "//avionics/firmware/drivers:septentrio",
        "//avionics/firmware/network:common",
        "//common:macros",
    ],
)
