package(default_visibility = ["//visibility:private"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/python/pack2:build_rules.bzl", "pack2")

makani_c_library(
    name = "gs02_plc_interface",
    srcs = [
        "gs02_plc_interface.c",
    ],
    hdrs = [
        "plc_interface.h",
    ],
    deps = [
        ":config_params_c",
        "//avionics/common:avionics_messages",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:network_config",
        "//avionics/common:pack_plc_messages",
        "//avionics/common:safety_codes",
        "//avionics/common:tether_convert",
        "//avionics/common:tether_message",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/params:param_util_c",
    ],
)

makani_c_library(
    name = "plc_interface",
    srcs = [
        "plc_interface.c",
    ],
    hdrs = [
        "plc_interface.h",
    ],
    deps = [
        ":config_params_c",
        "//avionics/common:avionics_messages",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:network_config",
        "//avionics/common:pack_plc_messages",
        "//avionics/common:safety_codes",
        "//avionics/common:tether_convert",
        "//avionics/common:tether_message",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/params:param_util_c",
    ],
)

makani_c_library(
    name = "winch_interface",
    srcs = [
        "winch_interface.c",
        "winch_interface.h",
    ],
    visibility = ["//avionics/cs/firmware:__pkg__"],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:network_config",
        "//avionics/common:pack_winch_messages",
        "//avionics/common:safety_codes",
        "//avionics/firmware/network:aio",
    ],
)

tms570_application(
    name = "gs02_plc_application.elf",
    srcs = [
        "main.c",
        "selftest.c",
        "selftest.h",
    ],
    deps = [
        ":config_params_c",
        ":gs02_plc_interface",
        ":winch_interface",
        "//avionics/common:aio_version",
        "//avionics/common:build_info",
        "//avionics/common:network_config",
        "//avionics/common:safety_codes",
        "//avionics/common:strings",
        "//avionics/common:tether_convert",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/params:param_util_c",
        "//avionics/firmware/serial",
        "//avionics/firmware/serial:aio_serial_params_c",
        "//avionics/firmware/serial:ground_io_serial_params_c",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_plc_gs02",
    ],
)

tms570_application(
    name = "tophat_plc_application.elf",
    srcs = [
        "main.c",
        "selftest.c",
        "selftest.h",
    ],
    deps = [
        ":config_params_c",
        ":plc_interface",
        ":winch_interface",
        "//avionics/common:aio_version",
        "//avionics/common:build_info",
        "//avionics/common:network_config",
        "//avionics/common:safety_codes",
        "//avionics/common:strings",
        "//avionics/common:tether_convert",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/params:param_util_c",
        "//avionics/firmware/serial",
        "//avionics/firmware/serial:aio_serial_params_c",
        "//avionics/firmware/serial:ground_io_serial_params_c",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_plc_tophat",
    ],
)

# Pack2 libraries.

pack2(
    name = "config_params",
    src = "config_params.p2",
)

# Parameter files.

[
    gen_param_file(
        src = "config_params.yaml",
        out = key + "_plc_config_params.bin",
        key = key,
    )
    for key in [
        "winch",
        "detwist_debug",
        "detwist_tophat",
        "detwist_test_winch",
        "detwist_bench",
        "gs02_01",
        "gs02_02",
    ]
]
