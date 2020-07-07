package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_binary",
    "makani_cc_library",
    "makani_cc_test",
)
load("//lib/bazel:py_rules.bzl", "py_binary")

makani_c_library(
    name = "aio",
    srcs = ["aio.c"],
    hdrs = ["aio.h"],
    deps = [
        ":aio_socket",
        ":clock",
        "//avionics/common:aio_version",
        "//avionics/common:avionics_messages",
        "//avionics/common:endian",
        "//avionics/common:network_config",
        "//avionics/common:pack_aio_header",
        "//avionics/network:cvt_all",
        "//avionics/network:message_type",
        "//common:macros",
        "//system:labels",
    ],
)

makani_c_library(
    name = "aio_client",
    srcs = ["aio_client.c"],
    hdrs = ["aio_client.h"],
    deps = [
        ":aio_client_stats",
        ":aio_socket",
        ":clock",
        ":netlink_socket",
        "//avionics/common:aio_version",
        "//avionics/common:pack_aio_header",
        "//avionics/network:aio_node_to_ip_address",
        "//avionics/network:message_info",
        "//avionics/network:message_type",
        "//common:macros",
    ],
)

makani_c_library(
    name = "aio_client_stats",
    hdrs = ["aio_client_stats.h"],
    deps = [":netlink_stats"],
)

makani_cc_library(
    name = "aio_interface",
    srcs = ["aio_interface.cc"],
    hdrs = ["aio_interface.h"],
    deps = [
        ":aio_client",
        ":aio_socket",
    ],
)

makani_c_library(
    name = "aio_socket",
    srcs = ["aio_socket.c"],
    hdrs = ["aio_socket.h"],
    deps = [
        "//avionics/common:network_config",
        "//avionics/network:message_type",
    ],
)

makani_c_library(
    name = "clock",
    srcs = ["clock.c"],
    hdrs = ["clock.h"],
)

makani_cc_library(
    name = "cvt_util",
    hdrs = ["cvt_util.h"],
    deps = [
        "//avionics/common:cvt",
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
    ],
)

makani_c_library(
    name = "netlink_socket",
    srcs = ["netlink_socket.c"],
    hdrs = ["netlink_socket.h"],
    copts = select({
        "//lib/bazel:k8_clang_mode": [
            "-Wno-cast-align",
            "-Wno-sign-conversion",
        ],
        "//conditions:default": [
            "-Wno-cast-align",
            "-Wno-cast-qual",
            "-Wno-const-qual",
            "-Wno-sign-conversion",
        ],
    }),
    deps = [
        ":netlink_stats",
        "//avionics/common:network_addresses",
    ],
)

makani_c_library(
    name = "netlink_stats",
    hdrs = ["netlink_stats.h"],
)

makani_cc_library(
    name = "q7_slow_status_types",
    hdrs = ["q7_slow_status_types.h"],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/network:aio_node",
    ],
)

makani_cc_library(
    name = "q7_slow_status",
    srcs = ["q7_slow_status.cc"],
    hdrs = ["q7_slow_status.h"],
    deps = [
        ":q7_slow_status_types",
        "//avionics/common:avionics_messages",
        "//avionics/common:build_info",
        "//avionics/common:pack_avionics_messages",
        "//avionics/common:strings",
        "//avionics/network:aio_node_to_ip_address",
        "//common:macros",
    ],
)

py_binary(
    name = "console",
    srcs = ["console.py"],
    tags = ["tms570_util"],
    deps = ["//avionics/common:aio"],
)

makani_cc_binary(
    name = "test_command",
    srcs = ["test_command.cc"],
    tags = ["tms570_util"],
    deps = [
        ":aio_interface",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_binary(
    name = "q7_slow_status_sender",
    srcs = ["q7_slow_status_sender.cc"],
    archs = [
        "q7",
        "x86",
    ],
    deps = [
        ":aio",
        ":q7_slow_status",
    ],
)

# Unit tests.

makani_cc_test(
    name = "aio_client_test",
    srcs = [
        "aio_client_test.cc",
    ],
    tags = ["exclusive"],
    deps = [
        ":aio_client",
    ],
)

makani_cc_test(
    name = "aio_interface_test",
    srcs = [
        "aio_interface_test.cc",
    ],
    tags = ["exclusive"],
    deps = [
        ":aio_interface",
    ],
)

makani_cc_test(
    name = "aio_socket_test",
    srcs = [
        "aio_socket_test.cc",
    ],
    tags = ["exclusive"],
    deps = [
        ":aio_socket",
        "//avionics/common:network_config",
        "//common:macros",
    ],
)

makani_cc_test(
    name = "netlink_socket_test",
    srcs = [
        "netlink_socket_test.cc",
    ],
    tags = ["exclusive"],
    deps = [
        ":netlink_socket",
        "//avionics/common:network_config",
    ],
)
