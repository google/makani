package(default_visibility = ["//visibility:private"])

load(
    "//avionics/firmware:build_rules.bzl",
    "tms570_application",
)
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/python/pack2:build_rules.bzl", "pack2")

# C libraries.

makani_c_library(
    name = "fc_library",
    srcs = ["main.c"] + glob([
        "fpv_pitot_control.*",
        "monitor.*",
        "output.*",
        "selftest.*",
    ]),
    visibility = [
        "//avionics/light/firmware:__subpackages__",
    ],
    deps = [
        ":config_params_c",
        "//avionics/common:build_info",
        "//avionics/common:controller_arbitration",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:endian",
        "//avionics/common:imu_output",
        "//avionics/common:novatel_types",
        "//avionics/common:pitot_cover_types",
        "//avionics/common:safety_codes",
        "//avionics/common:septentrio_types",
        "//avionics/common/fast_math",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:adis16488",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/drivers:faa_light",
        "//avionics/firmware/drivers:hsc",
        "//avionics/firmware/drivers:led",
        "//avionics/firmware/drivers:q7_watchdog",
        "//avionics/firmware/gps",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/monitors:analog",
        "//avionics/firmware/monitors:fc",
        "//avionics/firmware/monitors:mcp9800",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/serial",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_node",
        "//avionics/network:message_stats",
        "//avionics/network:message_type",
        "//common:macros",
    ],
)

# TMS570 applications.

tms570_application(
    name = "fc_application.elf",
    srcs = [],
    deps = [
        "fc_library",
        "//avionics/network:cvt_flight_computer",
    ],
)

# Pack2 libraries.

pack2(
    name = "config_params",
    src = "config_params.p2",
    deps = [
        "//avionics/common:gps_receiver",
        "//avionics/common:imu_config",
        "//avionics/firmware/drivers:hsc_model",
    ],
)

# Parameter files.
# TODO: These rules should probably go away once we have proper MES
# integration.

gen_param_file(
    src = "config_params.yaml",
    out = "nogps_fpv_HSC006MDSA3_fc_config_params.bin",
    key = "nogps_fpv_HSC006MDSA3",
)

gen_param_file(
    src = "config_params.yaml",
    out = "nogps_pitot_HSC006MDSA3_fc_config_params.bin",
    key = "nogps_pitot_HSC006MDSA3",
)

gen_param_file(
    src = "config_params.yaml",
    out = "novatel_HSC001PDSA3_fc_config_params.bin",
    key = "novatel_HSC001PDSA3",
)

gen_param_file(
    src = "config_params.yaml",
    out = "septentrio_HSC001PDSA3_fc_config_params.bin",
    key = "septentrio_HSC001PDSA3",
)
