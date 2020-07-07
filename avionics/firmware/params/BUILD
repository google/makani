package(default_visibility = ["//avionics:__subpackages__"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:autogen_rules.bzl", "message_ruleset")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library")
load("//lib/python/pack2:build_rules.bzl", "pack2")

# C libraries.

makani_c_library(
    name = "common",
    srcs = [
        "common.c",
        "param_section.c",
    ],
    hdrs = [
        "common.h",
        "param_section.h",
    ],
    visibility = ["//visibility:private"],
    deps = [
        ":param_header_c",
        ":param_types",
        "//avionics/common:crc",
        "//avionics/firmware/startup:ldscript",
        "//common:macros",
    ],
)

makani_c_library(
    name = "flash",
    srcs = ["flash.c"],
    hdrs = [
        "flash.h",
        "flash_internal.h",
    ],
    deps = [
        ":common",
    ],
)

makani_c_library(
    name = "i2c",
    srcs = ["i2c.c"],
    hdrs = [
        "i2c.h",
        "i2c_internal.h",
    ],
    deps = [
        ":common",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:eeprom",
        "//avionics/firmware/serial:serial_params_c",
    ],
)

makani_c_library(
    name = "param_types",
    hdrs = [
        "param_types.h",
    ],
    visibility = ["//avionics/common:__pkg__"],
)

makani_c_library(
    name = "params",
    hdrs = ["params.h"],
    deps = [
        ":flash",
        ":i2c",
    ],
)

makani_c_library(
    name = "params_server",
    srcs = [
        "flash_params_header.h",
        "param_server.c",
    ],
    hdrs = [
        "param_server.h",
    ],
    deps = [
        ":param_types",
        ":params",
        "//avionics/common:avionics_messages",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/network:common",
        "//common:macros",
    ],
)

makani_c_library(
    name = "param_util_c",
    srcs = ["param_util.c"],
    hdrs = ["param_util.h"],
    deps = [
        "//avionics/common:network_addresses",
    ],
)

# Generated code.

message_ruleset(
    name = "flash_params_header",
    generate_cvt_lib = False,
    header = "flash_params_header.h",
    visibility = ["//visibility:public"],
)

pack2(
    name = "param_header",
    src = "param_header.p2",
)

pack2(
    name = "param_source_header",
    src = "param_source_header.p2",
)

py_library(
    name = "common_py",
    srcs = [
        "client.py",
        "codec.py",
    ],
    deps = [
        ":param_header_py",
        ":param_source_header_py",
        "//avionics/common:aio",
        "//avionics/network:h2py_aio_node",
        "//lib/python/pack2:paramdb",
        requirement("PyYAML"),
    ],
)

py_binary(
    name = "param_util",
    srcs = ["param_util.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":common_py",
        "//lib/python:c_helpers",
        requirement("python-gflags"),
    ],
)
