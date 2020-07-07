package(default_visibility = ["//avionics:__subpackages__"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")
load("//lib/bazel:c_rules.bzl", "makani_c_library", "makani_cc_test")

# C libraries.

makani_c_library(
    name = "selection",
    srcs = ["selection.c"],
    hdrs = ["selection.h"],
    deps = [
        "//avionics/common:strings",
    ],
)

makani_c_library(
    name = "test",
    srcs = [
        "output.c",
        "output.h",
        "poll.c",
        "poll.h",
        "test.c",
        "test_main.c",
        "types.h",
    ],
    hdrs = [
        "test.h",
        "test_main.h",
    ],
    deps = [
        ":selection",
        "//avionics/common:build_info",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/cpu:registers",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/identity",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/startup:ldscript",
        "//avionics/network:switch_types",
        "//common:macros",
    ],
)

makani_c_library(
    name = "test_test",
    srcs = ["test_test.c"],
    hdrs = ["test_test.h"],
    deps = [
        ":test",
    ],
)

# Binaries.

tms570_application(
    name = "vim_test.elf",
    srcs = [
        "vim_main.c",
    ],
    deps = [
        "//avionics/common:network_config",
        "//avionics/common:strings",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/identity",
        "//avionics/firmware/network:aio",
        "//avionics/network:aio_labels",
        "//avionics/network:cvt_tms570s",
    ],
)

# Unit tests.

makani_cc_test(
    name = "selection_test",
    srcs = [
        "selection_test.cc",
    ],
    deps = [
        ":selection",
        "//avionics/common:strings",
    ],
)
