package(default_visibility = ["//visibility:private"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")
load("//lib/python/pack2:build_rules.bzl", "pack2")

tms570_application(
    name = "gps_application.elf",
    srcs = ["main.c"] + glob([
        "monitor.*",
        "output.*",
        "selftest.*",
    ]),
    deps = [
        ":config_params_c",
        "//avionics/common:avionics_messages",
        "//avionics/common:build_info",
        "//avionics/common:imu_output",
        "//avionics/common:serial_parse",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/drivers:faa_light",
        "//avionics/firmware/drivers:novatel",
        "//avionics/firmware/drivers:q7_watchdog",
        "//avionics/firmware/drivers:septentrio",
        "//avionics/firmware/gps",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/monitors:fc",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:common",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/serial",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_gps",
        "//avionics/network:message_stats",
        "//common:macros",
        "//common/c_math:common",
    ],
)

# Pack2 libraries.

pack2(
    name = "config_params",
    src = "config_params.p2",
    deps = [
        "//avionics/common:gps_receiver",
        "//avionics/common:imu_config",
    ],
)

# Parameter files.
# TODO: These rules should probably go away once we have proper MES
# integration.

gen_param_file(
    src = "config_params.yaml",
    out = "novatel_gps_config_params.bin",
    key = "novatel",
)

gen_param_file(
    src = "config_params.yaml",
    out = "septentrio_gps_config_params.bin",
    key = "septentrio",
)
