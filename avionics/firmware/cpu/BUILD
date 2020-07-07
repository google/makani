package(default_visibility = ["//avionics:__subpackages__"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/bazel:py_rules.bzl", "py_binary")

# C libraries.

makani_c_library(
    name = "clock",
    srcs = ["clock.c"],
    hdrs = ["clock.h"],
    deps = [
        ":registers",
        "//avionics/firmware/identity:board_hardware",
        "//avionics/firmware/serial:aio_serial_params_c",
        "//avionics/firmware/serial:board_serial",
        "//avionics/firmware/serial:cs_serial_params_c",
        "//avionics/firmware/serial:motor_serial_params_c",
        "//avionics/firmware/startup:common",
        "//common:macros",
    ],
)

makani_c_library(
    name = "peripherals",
    srcs = [
        "adc.c",
        "dcan.c",
        "dcan_pin.c",
        "emac.c",
        "gio.c",
        "i2c.c",
        "io.c",
        "iomm.c",
        "memcpy.S",
        "memory.c",
        "memory.h",
        "mibspi.c",
        "n2het.c",
        "peripherals.c",
        "rti.c",
        "sci.c",
        "sci_pin.c",
        "spi.c",
        "spi_pin.c",
        "swi.c",
        "vim.c",
        "watchdog.c",
    ],
    hdrs = [
        "adc.h",
        "dcan.h",
        "dcan_pin.h",
        "emac.h",
        "gio.h",
        "i2c.h",
        "io.h",
        "iomm.h",
        "memcpy.h",
        "mibspi.h",
        "mode.h",
        "n2het.h",
        "peripherals.h",
        "rti.h",
        "sci.h",
        "sci_pin.h",
        "spi.h",
        "spi_pin.h",
        "swi.h",
        "vim.h",
        "watchdog.h",
    ],
    undefined_symbols = [
        "OnFatalException",  # Handled by application.
    ],
    deps = [
        ":clock",
        ":registers",
        "//avionics/common:network_config",
        "//avionics/common:strings",
        "//avionics/firmware/startup:common",
        "//common:barrier",
        "//common:macros",
    ],
)

makani_c_library(
    name = "on_fatal_stub",
    srcs = ["on_fatal_stub.c"],
    hdrs = ["vim.h"],
)

makani_c_library(
    name = "registers",
    hdrs = [
        "registers.h",
        "registers_def.h",
    ],
)

# Python binaries.

py_binary(
    name = "generate_registers",
    srcs = ["registers.py"],
    main = "registers.py",
    deps = [
        "//lib/python:string_util",
        requirement("python-gflags"),
        requirement("PyYAML"),
    ],
)

# Code generation.

genrule(
    name = "registers_genrule",
    srcs = ["registers.yaml"],
    outs = [
        "registers.h",
        "registers_def.h",
    ],
    cmd = " ".join([
        "$(location :generate_registers)",
        "--definition_file",
        "$(location :registers.yaml)",
        "--struct_file",
        "$(location :registers.h)",
        "--define_file",
        "$(location :registers_def.h)",
        "--autogen_root",
        "$(GENDIR)",
    ]),
    tools = [":generate_registers"],
)

# Test functions.

makani_c_library(
    name = "adc_test",
    srcs = ["adc_test.c"],
    hdrs = ["adc_test.h"],
    deps = [
        ":peripherals",
        "//avionics/common:endian",
        "//avionics/firmware/test",
    ],
)

makani_c_library(
    name = "io_test",
    srcs = ["io_test.c"],
    hdrs = ["io_test.h"],
    deps = [
        ":peripherals",
        "//avionics/firmware/test",
    ],
)

makani_c_library(
    name = "memcpy_test",
    srcs = ["memcpy_test.c"],
    hdrs = ["memcpy_test.h"],
    deps = [
        ":peripherals",
        "//avionics/firmware/test",
    ],
)
