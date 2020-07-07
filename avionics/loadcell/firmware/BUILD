package(default_visibility = ["//visibility:private"])

load(
    "//avionics/firmware:build_rules.bzl",
    "tms570_application",
    "tms570_test",
)
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/python/pack2:build_rules.bzl", "pack2")

exports_files(["calib_params.yaml"])

# C libraries

makani_c_library(
    name = "tether_release",
    srcs = ["tether_release.c"],
    hdrs = ["tether_release.h"],
    deps = [
        "//avionics/common:safety_codes",
        "//avionics/firmware/cpu:peripherals",
    ],
)

# C binaries.

tms570_application(
    name = "loadcell_application.elf",
    srcs = [
        "bridle_junc_can.c",
        "bridle_junc_can.h",
        "input.c",
        "input.h",
        "loadcell.c",
        "loadcell.h",
        "main.c",
        "mon.c",
        "mon.h",
        "output.c",
        "output.h",
        "selftest.c",
        "selftest.h",
    ],
    deps = [
        ":calib_params_c",
        ":tether_release",
        "//avionics/common:build_info",
        "//avionics/common:controller_arbitration",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:loadcell_types",
        "//avionics/common:network_config",
        "//avionics/common:safety_codes",
        "//avionics/common:tether_message",
        "//avionics/common:tether_message_types",
        "//avionics/common/fast_math",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:ad7192",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/monitors:loadcell",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/serial",
        "//avionics/firmware/serial:aio_serial_params_c",
        "//avionics/firmware/serial:carrier_serial",
        "//avionics/firmware/serial:loadcell_serial_params_c",
        "//avionics/firmware/util:selftest",
        "//avionics/firmware/util:timer",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_loadcell_node",
        "//avionics/network:message_stats",
        "//avionics/network:message_type",
        "//common:macros",
    ],
)

tms570_test(
    name = "loadcell_test_application.elf",
    srcs = [
        "test_main.c",
        "tether_release_test.c",
        "tether_release_test.h",
    ],
    deps = [
        ":tether_release",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/identity",
        "//avionics/network:cvt_loadcell_node",
    ],
)

# Pack2 libraries.

pack2(
    name = "calib_params",
    src = "calib_params.p2",
)

# Parameter files.

[
    gen_param_file(
        src = "calib_params.yaml",
        out = key + "_loadcell_calib_params.bin",
        key = key,
    )
    for key in [
        "port_sn1",
        "port_sn4",
        "port_sn5",
        "port_sn6",
        "port_sn7",
        "starboard_sn1",
        "starboard_sn4",
        "starboard_sn5",
        "starboard_sn6",
        "starboard_sn7",
    ]
]
