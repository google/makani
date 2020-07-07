package(default_visibility = ["//visibility:private"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")

tms570_application(
    name = "short_stack_application.elf",
    srcs = glob([
        "*.c",
        "*.h",
    ]),
    deps = [
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
        "//avionics/firmware/monitors:short_stack",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/serial",
        "//avionics/firmware/serial:aio_serial_params_c",
        "//avionics/firmware/serial:short_stack_serial_params_c",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_short_stack",
    ],
)
