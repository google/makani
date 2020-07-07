package(default_visibility = ["//avionics:__subpackages__"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:autogen_rules.bzl", "message_ruleset")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/bazel:py_rules.bzl", "py_binary")

# C libraries.

makani_c_library(
    name = "common",
    srcs = [
        "net.c",
        "net_aio.h",
        "net_diag.c",
        "net_probe.c",
        "net_route_diag.c",
        "net_send.c",
        "pack_net_probe_message.c",
    ],
    hdrs = [
        "net.h",
        "net_diag.h",
        "net_probe.h",
        "net_route_diag.h",
        "net_send.h",
        "pack_net_probe_message.h",
    ],
    undefined_symbols = [
        "HandleAioFrame",
    ],
    deps = [
        ":net_probe_message",
        "//avionics/common:aio_header",
        "//avionics/common:aio_version",
        "//avionics/common:cvt",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:endian",
        "//avionics/common:network_config",
        "//avionics/common:pack_aio_header",
        "//avionics/common:pack_avionics_messages",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/identity",
        "//avionics/firmware/identity:identity_types",
        "//avionics/network:aio_node",
        "//avionics/network:aio_node_to_ip_address",
        "//avionics/network:message_type",
        "//avionics/network:switch_config",
        "//common:macros",
    ],
)

makani_c_library(
    name = "aio",
    srcs = [
        "net_aio.c",
    ],
    hdrs = [],
    deps = [
        ":common",
        "//avionics/common:cvt",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
    ],
)

makani_c_library(
    name = "aio_stub",
    srcs = [
        "net_aio_stub.c",
    ],
    hdrs = [],
    deps = [
        ":common",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
    ],
)

makani_c_library(
    name = "net_mon",
    srcs = ["net_mon.c"],
    hdrs = ["net_mon.h"],
    deps = [
        ":common",
        ":net_mon_types",
        "//avionics/firmware/drivers:bcm",
        "//avionics/network:switch_types",
        "//common:macros",
    ],
)

makani_c_library(
    name = "net_mon_types",
    hdrs = ["net_mon_types.h"],
    deps = [
        "//avionics/common:network_config",
    ],
)

message_ruleset(
    name = "net_probe_message",
    generate_cvt_lib = False,
    header = "net_probe_message.h",
)

makani_c_library(
    name = "switch_config",
    srcs = ["switch_config.c"],
    hdrs = ["switch_config.h"],
    deps = [
        "//avionics/firmware/identity",
        "//avionics/network:routes",
    ],
)

# Code generation rules.

py_binary(
    name = "generate_net_send",
    srcs = ["generate_net_send.py"],
    deps = [
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/network:h2py_message_type",
        "//lib/python:c_helpers",
        requirement("python-gflags"),
    ],
)

genrule(
    name = "net_send_genrule",
    outs = [
        "net_send.c",
        "net_send.h",
    ],
    cmd = " ".join([
        "$(location :generate_net_send)",
        "--source_file=$(location net_send.c)",
        "--header_file=$(location net_send.h)",
    ]),
    tools = [":generate_net_send"],
)
