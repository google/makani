package(default_visibility = ["//avionics/firmware/cpu:__pkg__"])

load("//avionics/firmware/startup:build_rules.bzl", "startup_c_library")
load("//lib/bazel:autogen_rules.bzl", "message_ruleset")
load("//lib/bazel:c_rules.bzl", "makani_c_library")

# C libraries.

makani_c_library(
    name = "config",
    srcs = [
        "clock_tms570_config.c",
        "clock_tms570_config.h",
    ],
    deps = [
        "//common:macros",
    ],
)

makani_c_library(
    name = "common",
    srcs = [
        "clock_tms570.S",
        "cpu_tms570.S",
        "efuse_tms570.S",
        "esm_tms570.S",
        "flash_tms570.S",
        "peripherals_tms570.S",
        "ram_tms570.S",
        "return_codes.h",
    ],
    hdrs = [
        "clock_tms570.h",
        "cpu_tms570.h",
        "flash_tms570.h",
    ],
    deps = [
        ":config",
        ":ldscript",
        "//avionics/firmware/cpu:registers",
        "//common:macros",
    ],
)

makani_c_library(
    name = "ldscript",
    srcs = [
        "ldscript.c",
        "ldscript_types.h",
    ],
    hdrs = [
        "ldscript.h",
    ],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "newlib",
    hdrs = ["newlib.h"],
    visibility = ["//avionics:__subpackages__"],
)

makani_c_library(
    name = "vectors_app",
    srcs = ["vectors_app_tms570.S"],
    undefined_symbols = [
        "InterruptDataAbortHandler",
        "InterruptPrefetchAbortHandler",
        "InterruptSoftwareInterruptHandler",
        "InterruptUndefInstructionHandler",
        "StartupClockInit",
        "StartupRtiInit",
        "StartupVimInit",
        "main",
    ],
    visibility = ["//avionics:__subpackages__"],
    deps = [
        ":common",
        "//avionics/firmware/cpu:registers",
    ],
)

makani_c_library(
    name = "vectors_bootloader",
    srcs = ["vectors_bootloader_tms570.S"],
    undefined_symbols = [
        "InterruptDataAbortHandler",
        "InterruptPrefetchAbortHandler",
        "InterruptSoftwareInterruptHandler",
        "InterruptUndefInstructionHandler",
        "StartupClockInit",
        "StartupRtiInit",
        "StartupVimInit",
        "main",
    ],
    visibility = ["//avionics:__subpackages__"],
    deps = [
        ":common",
        "//avionics/firmware/cpu:registers",
    ],
)

# C startup libraries.

startup_c_library(
    name = "application",
    ld_files = [
        "1_chip_tms570_ls1227.ld",
        "2_slot_app_tms570.ld",
        "3_common_tms570.ld",
        "4_padding_app_tms570.ld",
    ],
    visibility = ["//avionics:__subpackages__"],
    deps = [
        ":common",
        ":vectors_app",
    ],
)

startup_c_library(
    name = "application_ram",
    ld_files = [
        "1_chip_tms570_ls1227.ld",
        "2_slot_app_ram_tms570.ld",
        "3_common_tms570.ld",
        "4_padding_app_tms570.ld",
    ],
    visibility = ["//avionics:__subpackages__"],
    deps = [
        ":common",
        ":vectors_app",
    ],
)

startup_c_library(
    name = "bootloader",
    ld_files = [
        "1_chip_tms570_ls1227.ld",
        "2_slot_bootloader_tms570.ld",
        "3_common_tms570.ld",
        "4_padding_bootloader_tms570.ld",
    ],
    visibility = ["//avionics:__subpackages__"],
    deps = [
        ":common",
        ":vectors_bootloader",
    ],
)

# Python libraries.

message_ruleset(
    name = "ldscript_types",
    generate_cvt_lib = False,
    generate_h2py_lib = True,
    header = "ldscript_types.h",
    visibility = ["//avionics:__subpackages__"],
)

# This filegroup is needed to mount linker scripts in the sandbox. See
# https://github.com/bazelbuild/bazel/issues/807.
filegroup(
    name = "linker_scripts",
    srcs = glob(["*.ld"]),
    visibility = ["//lib/bazel:__pkg__"],
)
