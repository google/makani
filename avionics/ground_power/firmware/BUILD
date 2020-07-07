package(default_visibility = ["//visibility:private"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")

tms570_application(
    name = "ground_power_application.elf",
    srcs = [
        "main.c",
        "selftest.c",
        "selftest.h",
    ],
    deps = [
        "//avionics/common:aio_version",
        "//avionics/common:build_info",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:network_config",
        "//avionics/common:strings",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:eeprom",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_ground_power_tms570",
    ],
)
