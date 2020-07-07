package(default_visibility = ["//avionics:__subpackages__"])

load("//lib/bazel:c_rules.bzl", "makani_c_library")

makani_c_library(
    name = "assert",
    srcs = ["assert.c"],
    deps = [
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "newlib",
    srcs = [
        "newlib_syscalls.c",
    ],
    hdrs = [],
    undefined_symbols = [
        "_close",
        "_fstat",
        "_getpid",
        "_isatty",
        "_kill",
        "_lseek",
        "_read",
        "_sbrk",
        "_write",
    ],
    deps = [
        "//avionics/common:network_config",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/network:common",
        "//avionics/firmware/startup:newlib",
        "//avionics/network:aio_node",
    ],
)

makani_c_library(
    name = "selftest",
    srcs = [
        "selftest.c",
    ],
    hdrs = [
        "selftest.h",
    ],
    deps = [
        "//avionics/common:build_info",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:eeprom",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/identity",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/params",
        "//avionics/firmware/serial",
        "//avionics/network:aio_node",
        "//avionics/network:switch_types",
    ],
)

makani_c_library(
    name = "state_machine",
    srcs = ["state_machine.c"],
    hdrs = ["state_machine.h"],
    deps = [
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "timer",
    srcs = ["timer.c"],
    hdrs = ["timer.h"],
    deps = [
        "//avionics/firmware/cpu:clock",
    ],
)
