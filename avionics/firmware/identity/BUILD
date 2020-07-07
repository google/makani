package(default_visibility = ["//avionics:__subpackages__"])

load(
    "//lib/bazel:autogen_rules.bzl",
    "h2py_library",
)
load("//lib/bazel:c_rules.bzl", "makani_c_library")

# C libraries.

makani_c_library(
    name = "board_hardware",
    srcs = ["board_hardware.c"],
    hdrs = ["board_hardware.h"],
    deps = [
        ":identity_types",
        "//avionics/firmware/startup:ldscript",
    ],
)

makani_c_library(
    name = "carrier_hardware",
    srcs = ["carrier_hardware.c"],
    hdrs = ["carrier_hardware.h"],
    deps = [
        ":identity_types",
        "//avionics/firmware/cpu:peripherals",
        "//common:macros",
    ],
)

makani_c_library(
    name = "identity",
    srcs = [
        "identity.c",
    ],
    hdrs = [
        "identity.h",
    ],
    deps = [
        ":identity_types",
        "//avionics/common:network_addresses",
        "//avionics/firmware/startup:ldscript",
        "//avionics/network:aio_node",
        "//avionics/network:aio_node_to_ip_address",
    ],
)

makani_c_library(
    name = "identity_types",
    hdrs = ["identity_types.h"],
)

# Python/h2py rules.

h2py_library(
    name = "h2py_identity_types",
    function_deps = [":identity_types"],
    header = "identity_types.h",
    visibility = ["//visibility:public"],
)
