package(default_visibility = ["//visibility:public"])

load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load("//lib/bazel:c_rules.bzl", "makani_c_binary", "makani_c_library")

makani_c_library(
    name = "ground_power_types",
    srcs = ["ground_power_types.c"],
    hdrs = ["ground_power_types.h"],
    deps = [
        ":inverter",
        ":loadbank",
    ],
)

makani_c_library(
    name = "inverter",
    srcs = ["inverter.c"],
    hdrs = [
        "inverter.h",
        "inverter_types.h",
    ],
    deps = [
        ":flags",
        "//avionics/common:avionics_messages",
        "//avionics/common:pack_avionics_messages",
        "//avionics/linux:aio",
        "//common:macros",
        "//third_party/libmodbus:modbus",
    ],
)

makani_c_library(
    name = "loadbank",
    srcs = ["loadbank.c"],
    hdrs = [
        "loadbank.h",
        "loadbank_types.h",
    ],
    deps = [
        "//avionics/common:avionics_messages",
        "//common:macros",
        "//third_party/libmodbus:modbus",
    ],
)

h2py_library(
    name = "h2py_inverter_types",
    header = "inverter_types.h",
    header_deps = [
        "//avionics/common:avionics_messages",
    ],
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_flags",
    header = "flags.h",
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "flags",
    hdrs = ["flags.h"],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "ground_power",
    srcs = ["ground_power.c"],
    hdrs = ["ground_power.h"],
    linkopts = ["-lrt"],
    deps = [
        ":avionics_interface",
        ":ground_power_types",
        "//avionics/common:avionics_messages",
        "//avionics/common:pack_avionics_messages",
        "//avionics/linux:aio",
        "//avionics/network:message_type",
    ],
)

makani_c_library(
    name = "avionics_interface",
    srcs = ["avionics_interface.c"],
    hdrs = ["avionics_interface.h"],
    linkopts = ["-lrt"],
    deps = [
        ":flags",
        ":ground_power_types",
        ":inverter",
        "//avionics/common:avionics_messages",
        "//avionics/common:cvt_avionics_messages",
    ],
)

makani_c_binary(
    name = "ground_power_monitor",
    srcs = ["ground_power_monitor.c"],
    archs = ["q7"],
    copts = [
        "-std=c99",
        "-w",
    ],
    linkopts = ["-pthread"],
    deps = [
        ":ground_power",
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//common:macros",
    ],
)
