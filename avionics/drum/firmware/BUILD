package(default_visibility = ["//visibility:private"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/python/pack2:build_rules.bzl", "pack2")

# C libraries.

makani_c_library(
    name = "encoders",
    srcs = ["encoders.c"],
    hdrs = ["encoders.h"],
    deps = [
        ":config_params_c",
        "//avionics/common:avionics_messages",
        "//avionics/common:faults",
        "//avionics/firmware/drivers:encoder",
        "//avionics/firmware/identity",
    ],
)

# C binaries.

tms570_application(
    name = "drum_application.elf",
    srcs = [
        "main.c",
        "output.c",
        "output.h",
        "selftest.c",
        "selftest.h",
    ],
    deps = [
        ":config_params_c",
        ":encoders",
        "//avionics/common:build_info",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/monitors:ground_io",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/serial",
        "//avionics/firmware/serial:aio_serial_params_c",
        "//avionics/firmware/serial:ground_io_serial_params_c",
        "//avionics/firmware/serial:servo_serial_params_c",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_drum",
    ],
)

# Pack2 libraries.

pack2(
    name = "config_params",
    src = "config_params.p2",
    deps = [
        "//avionics/firmware/drivers:encoder_params",
    ],
)

# Parameter files.
# TODO: These rules should probably go away once we have proper MES
# integration.

[
    gen_param_file(
        src = "config_params.yaml",
        out = key + "_drum_config_params.bin",
        key = key,
    )
    for key in [
        "gsv2_01_a",
        "gsv2_01_b",
        "gsv2_02_a",
        "gsv2_02_b",
    ]
]
