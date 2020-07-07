package(default_visibility = ["//visibility:private"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")

tms570_application(
    name = "recorder_application.elf",
    srcs = glob([
        "*.c",
        "*.h",
    ]),
    deps = [
        "//avionics/common:actuator_types",
        "//avionics/common:build_info",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:network_config",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:q7_watchdog",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/monitors:recorder",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/serial",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_recorder_tms570",
        "//avionics/network:message_type",
        "//common:macros",
    ],
)
