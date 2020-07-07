package(default_visibility = ["//visibility:public"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/python/pack2:build_rules.bzl", "pack2")

# C libraries.

makani_c_library(
    name = "eop_interface",
    srcs = ["eop_interface.c"],
    hdrs = ["eop_interface.h"],
    deps = [
        "//avionics/common:eop_version",
        "//avionics/common:network_config",
        "//avionics/common:pack_avionics_messages",
        "//avionics/common:pack_eop_messages",
        "//avionics/firmware/identity",
        "//avionics/firmware/network:aio",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
    ],
)

makani_c_library(
    name = "tether_rtcm",
    srcs = [
        "tether_rtcm.c",
    ],
    hdrs = [
        "tether_rtcm.h",
    ],
    deps = [
        "//avionics/common:aio_header",
        "//avionics/common:avionics_messages",
        "//avionics/common:rtcm3",
        "//avionics/common:serial_parse",
        "//avionics/common:tether_message_types",
        "//avionics/firmware/gps",
        "//common:macros",
    ],
)

makani_c_library(
    name = "tether_comms",
    srcs = [
        "tether_comms.c",
    ],
    hdrs = [
        "tether_comms.h",
    ],
    deps = [
        ":tether_rtcm",
        "//avionics/common:controller_arbitration",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:gill_types",
        "//avionics/common:novatel_types",
        "//avionics/common:pack_tether_message",
        "//avionics/common:safety_codes",
        "//avionics/common:tether_cvt",
        "//avionics/common:tether_message",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:xlr",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon_types",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_stats",
        "//common:macros",
    ],
)

# C binaries.

tms570_application(
    name = "cs_application.elf",
    srcs = [
        "main.c",
        "monitor.c",
        "monitor.h",
        "output.c",
        "output.h",
        "selftest.c",
        "selftest.h",
    ],
    deps = [
        ":config_params_c",
        ":eop_interface",
        ":tether_comms",
        "//avionics/common:actuator_types",
        "//avionics/common:build_info",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:network_config",
        "//avionics/common:pack_winch_messages",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:microhard",
        "//avionics/firmware/drivers:pca9546a",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:analog",
        "//avionics/firmware/monitors:cs",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/serial",
        "//avionics/firmware/serial:cs_serial_params_c",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_latency",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_core_switch",
        "//avionics/network:message_type",
        "//avionics/plc/firmware:winch_interface",
        "//common:macros",
    ],
)

tms570_application(
    name = "cs_test_fixture_application.elf",
    srcs = [
        "test_fixture_main.c",
    ],
    deps = [
        "//avionics/common:network_config",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/identity",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/serial",
        "//avionics/network:cvt_core_switch",
        "//common:macros",
    ],
)

pack2(
    name = "config_params",
    src = "config_params.p2",
    deps = ["//avionics/firmware/drivers:microhard_params"],
)

# Parameter files.
[
    gen_param_file(
        src = "config_params.yaml",
        out = key + "_cs_config_params.bin",
        key = key,
    )
    for key in [
        "a",
        "b",
        "gs_a",
        "gs_b",
        "noradio_a",
        "noradio_b",
        "noradio_gs_a",
        "noradio_gs_b",
    ]
]
