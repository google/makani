package(default_visibility = ["//visibility:private"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")
load("//lib/python/pack2:build_rules.bzl", "pack2")

tms570_application(
    name = "batt_application.elf",
    srcs = glob([
        "*.c",
        "*.h",
    ]),
    deps = [
        ":config_params_c",
        "//avionics/common:aio_version",
        "//avionics/common:build_info",
        "//avionics/common:network_config",
        "//avionics/common:safety_codes",
        "//avionics/common:strings",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:log",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/monitors:batt",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/serial",
        "//avionics/firmware/serial:aio_serial_params_c",
        "//avionics/firmware/serial:batt_serial_params_c",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_batt",
    ],
)

pack2(
    name = "config_params",
    src = "config_params.p2",
)

[
    gen_param_file(
        src = "config_params.yaml",
        out = key + "_batt_config_params.bin",
        key = key,
    )
    for key in [
        "default",
        "dual_big",
    ]
]
