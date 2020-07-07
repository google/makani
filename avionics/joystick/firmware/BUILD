package(default_visibility = ["//visibility:private"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")
load("//lib/python/pack2:build_rules.bzl", "pack2")

tms570_application(
    name = "joystick_application.elf",
    srcs = [
        "ecap_bits.h",
        "main.c",
        "monitor.c",
        "monitor.h",
        "output.c",
        "output.h",
        "ppm.c",
        "ppm.h",
        "selftest.c",
        "selftest.h",
        "switches.c",
        "switches.h",
    ],
    deps = [
        ":calib_params_c",
        ":config_params_c",
        "//avionics/common:aio_version",
        "//avionics/common:build_info",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:debounce",
        "//avionics/common:network_config",
        "//avionics/common:safety_codes",
        "//avionics/common:strings",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:eeprom",
        "//avionics/firmware/drivers:microhard",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/monitors:ground_io",
        "//avionics/firmware/monitors:joystick",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/params:params_server",
        "//avionics/firmware/serial",
        "//avionics/firmware/serial:joystick_serial_params_c",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_joystick",
    ],
)

pack2(
    name = "calib_params",
    src = "calib_params.p2",
)

pack2(
    name = "config_params",
    src = "config_params.p2",
    deps = ["//avionics/firmware/drivers:microhard_params"],
)

# Parameter files.
# TODO: These rules should probably go away once we have proper MES
# integration.

gen_param_file(
    src = "calib_params.yaml",
    out = "futaba_t7c_joystick_calib_params.bin",
    key = "futaba_t7c",
)

gen_param_file(
    src = "config_params.yaml",
    out = "a_joystick_config_params.bin",
    key = "a",
)
