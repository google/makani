package(default_visibility = ["//visibility:public"])

load("//avionics/firmware:build_rules.bzl", "tms570_bootloader", "tms570_bootloader_application")
load(
    "//lib/bazel:autogen_rules.bzl",
    "h2py_library",
)
load("//lib/bazel:c_rules.bzl", "makani_c_library")

# C libraries.

makani_c_library(
    name = "bootloader_application_main",
    srcs = ["bootloader_application_main.c"],
    deps = [
        ":update_server",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/identity",
        "//avionics/firmware/network:aio_stub",
        "//avionics/firmware/output:slow_status",
        "//avionics/network:aio_node",
    ],
)

makani_c_library(
    name = "bootloader_main",
    srcs = ["bootloader_main.c"],
    deps = [
        ":update_server",
        "//avionics/common:crc",
        "//avionics/common:endian",
        "//avionics/common:network_config",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/identity",
        "//avionics/firmware/network:aio_stub",
        "//avionics/firmware/output:slow_status",
        "//common:macros",
    ],
)

makani_c_library(
    name = "flash",
    srcs = [
        "eeprom_flash.c",
        "flash.c",
        "internal_flash.c",
    ],
    hdrs = [
        "eeprom_flash.h",
        "flash.h",
        "internal_flash.h",
    ],
    deps = [
        "//avionics/firmware/drivers:eeprom",
        "//avionics/firmware/drivers:ext_watchdog_header",
        "//avionics/firmware/startup:ldscript",
        "//common:macros",
        "//third_party/f021",
    ],
)

makani_c_library(
    name = "update_server",
    srcs = ["update_server.c"],
    hdrs = ["update_server.h"],
    deps = [
        ":flash",
        "//avionics/common:crc",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/identity:identity_types",
        "//avionics/firmware/network:common",
    ],
)

# Python/h2py rules.

h2py_library(
    name = "h2py_update_server",
    header = "update_server.h",
    header_deps = ["//avionics/firmware/identity:identity_types"],
)

# Executable files.

tms570_bootloader(
    name = "bootloader.elf",
    deps = [
        "//avionics/firmware/drivers:ext_watchdog",
    ],
)

tms570_bootloader_application(
    name = "bootloader_application.elf",
    deps = [
        "//avionics/firmware/drivers:ext_watchdog",
    ],
)
