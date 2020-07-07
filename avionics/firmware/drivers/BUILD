package(default_visibility = ["//avionics:__subpackages__"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load("//lib/bazel:c_rules.bzl", "makani_c_library", "makani_cc_test")
load("//lib/bazel:py_rules.bzl", "py_binary")
load("//lib/python/pack2:build_rules.bzl", "pack2")

# C libraries.

makani_c_library(
    name = "ad7192",
    srcs = ["ad7192.c"],
    hdrs = ["ad7192.h"],
    deps = [
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/util:state_machine",
        "//common:macros",
    ],
)

makani_c_library(
    name = "ad7265",
    srcs = ["ad7265.c"],
    hdrs = ["ad7265.h"],
    deps = [
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "adis16488",
    srcs = [
        "adis16488.c",
        "adis16488_coef.c",
        "adis16488_coef.h",
        "adis16488_reg.h",
    ],
    hdrs = [
        "adis16488.h",
    ],
    deps = [
        "//avionics/common:adis16488_types",
        "//avionics/firmware/cpu:peripherals",
        "//common/c_math:common",
    ],
)

makani_c_library(
    name = "ads7828",
    srcs = ["ads7828.c"],
    hdrs = ["ads7828.h"],
    deps = [
        ":ads7828_types",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "ads7828_types",
    srcs = ["ads7828_types.c"],
    hdrs = ["ads7828_types.h"],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "ars308",
    srcs = ["ars308.c"],
    hdrs = ["ars308.h"],
    deps = [
        "//avionics/common:ars308_types",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "bcm",
    srcs = [
        "bcm.c",
        "bcm53101.c",
        "bcm53284.c",
        "bcm5482s.c",
        "bcm_unified.c",
    ],
    hdrs = [
        "bcm.h",
        "bcm53101.h",
        "bcm53284.h",
        "bcm5482s.h",
        "bcm_unified.h",
    ],
    deps = [
        "//avionics/common:endian",
        "//avionics/common:network_config",
        "//avionics/common:network_diag_types",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/util:state_machine",
        "//avionics/network:routes",
        "//avionics/network:switch_config",
        "//avionics/network:switch_types",
        "//common:macros",
    ],
)

makani_c_library(
    name = "bq34z100",
    srcs = ["bq34z100.c"],
    hdrs = ["bq34z100.h"],
    deps = [
        ":bq34z100_types",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "bq34z100_types",
    srcs = ["bq34z100_types.c"],
    hdrs = ["bq34z100_types.h"],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "canopen",
    srcs = [
        "canopen_nmt.c",
        "canopen_pdo.c",
        "canopen_sdo.c",
    ],
    hdrs = [
        "canopen_nmt.h",
        "canopen_pdo.h",
        "canopen_sdo.h",
    ],
    deps = [
        ":log",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "dac104s085",
    srcs = ["dac104s085.c"],
    hdrs = ["dac104s085.h"],
    deps = [
        "//avionics/common/fast_math",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "eeprom",
    srcs = ["eeprom24.c"],
    hdrs = ["eeprom24.h"],
    deps = [
        "//avionics/firmware/cpu:peripherals",
        "//common:macros",
    ],
)

makani_c_library(
    name = "encoder",
    srcs = ["encoder.c"],
    hdrs = ["encoder.h"],
    deps = [
        ":encoder_params_c",
        ":ssi_encoder",
        "//avionics/common/fast_math",
        "//common:macros",
    ],
)

makani_c_library(
    name = "ext_watchdog",
    srcs = ["ext_watchdog.c"],
    hdrs = ["ext_watchdog.h"],
    deps = [
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "ext_watchdog_header",
    hdrs = ["ext_watchdog.h"],
    undefined_symbols = [
        "ExtWatchdogInit",
        "ExtWatchdogKick",
        "ExtWatchdogPoll",
        "ExtWatchdogWaitCycles",
        "ExtWatchdogWaitUsec",
        "ExtWatchdogWaitMsec",
    ],
)

makani_c_library(
    name = "faa_light",
    srcs = ["faa_light.c"],
    hdrs = ["faa_light.h"],
    deps = [
        "//avionics/common:faa_light_types",
        "//avionics/common/fast_math",
        "//avionics/firmware/gps",
        "//avionics/firmware/network:common",
    ],
)

makani_c_library(
    name = "gps_device",
    srcs = ["gps_device.c"],
    hdrs = ["gps_device.h"],
    deps = [
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "hsc",
    srcs = ["hsc.c"],
    hdrs = ["hsc.h"],
    deps = [
        ":hsc_model_c",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
        "//common:macros",
    ],
)

makani_c_library(
    name = "ina219",
    srcs = ["ina219.c"],
    hdrs = ["ina219.h"],
    deps = [
        ":ina219_types",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "ina219_types",
    srcs = ["ina219_types.c"],
    hdrs = ["ina219_types.h"],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "led",
    srcs = ["led.c"],
    hdrs = ["led.h"],
    deps = [
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "log",
    srcs = ["log.c"],
    hdrs = ["log.h"],
    deps = [
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/startup:ldscript",
    ],
)

makani_c_library(
    name = "ltc2309",
    srcs = ["ltc2309.c"],
    hdrs = ["ltc2309.h"],
    deps = [
        ":ltc2309_types",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "ltc2309_types",
    srcs = ["ltc2309_types.c"],
    hdrs = ["ltc2309_types.h"],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "ltc4151",
    srcs = ["ltc4151.c"],
    hdrs = ["ltc4151.h"],
    deps = [
        ":ltc4151_types",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "ltc4151_types",
    srcs = ["ltc4151_types.c"],
    hdrs = ["ltc4151_types.h"],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "ltc6804",
    srcs = ["ltc6804.c"],
    hdrs = ["ltc6804.h"],
    deps = [
        ":ltc6804_types",
        "//avionics/common:endian",
        "//avionics/common/fast_math",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "ltc6804_types",
    hdrs = ["ltc6804_types.h"],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "mcp342x",
    srcs = ["mcp342x.c"],
    hdrs = ["mcp342x.h"],
    deps = [
        ":mcp342x_types",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "mcp342x_types",
    srcs = ["mcp342x_types.c"],
    hdrs = ["mcp342x_types.h"],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "mcp9800",
    srcs = ["mcp9800.c"],
    hdrs = ["mcp9800.h"],
    deps = [
        ":mcp9800_types",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "mcp9800_types",
    srcs = ["mcp9800_types.c"],
    hdrs = ["mcp9800_types.h"],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "metpak",
    srcs = ["metpak.c"],
    hdrs = ["metpak.h"],
    deps = [
        "//avionics/common:gill",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "microhard",
    srcs = ["microhard.c"],
    hdrs = ["microhard.h"],
    deps = [
        ":microhard_params_c",
        ":microhard_types",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/util:timer",
    ],
)

makani_c_library(
    name = "microhard_types",
    hdrs = ["microhard_types.h"],
    visibility = ["//visibility:public"],
)

makani_c_library(
    name = "novatel",
    srcs = [
        "novatel.c",
        "novatel_def.h",
        "novatel_recv.c",
    ],
    hdrs = [
        "novatel.h",
        "novatel_recv.h",
    ],
    deps = [
        ":gps_device",
        "//avionics/common:novatel",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "pca9546a",
    srcs = ["pca9546a.c"],
    hdrs = ["pca9546a.h"],
    deps = [
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "q7_watchdog",
    srcs = ["q7_watchdog.c"],
    hdrs = ["q7_watchdog.h"],
    deps = [
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "septentrio",
    srcs = [
        "septentrio.c",
        "septentrio_def.h",
    ],
    hdrs = [
        "septentrio.h",
    ],
    deps = [
        ":gps_device",
        "//avionics/common:septentrio",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "si7021",
    srcs = ["si7021.c"],
    hdrs = ["si7021.h"],
    deps = [
        ":si7021_types",
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "si7021_types",
    srcs = ["si7021_types.c"],
    hdrs = ["si7021_types.h"],
    visibility = ["//visibility:public"],
    deps = ["//avionics/common/fast_math"],
)

makani_c_library(
    name = "spi_flash",
    srcs = ["spi_flash.c"],
    hdrs = ["spi_flash.h"],
    deps = [
        "//avionics/common:endian",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "ssi_encoder",
    srcs = ["ssi_encoder.c"],
    hdrs = ["ssi_encoder.h"],
    deps = [
        "//avionics/common:encoder_types",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "windmaster",
    srcs = ["windmaster.c"],
    hdrs = ["windmaster.h"],
    deps = [
        "//avionics/common:gill",
        "//avionics/firmware/cpu:peripherals",
    ],
)

makani_c_library(
    name = "xlr",
    srcs = [
        "xlr.c",
        "xlr_config.c",
    ],
    hdrs = [
        "xlr.h",
        "xlr_config.h",
    ],
    deps = [
        ":xlr_serial",
        "//avionics/firmware/cpu:peripherals",
        "//common:macros",
        "//common/c_math:common",
    ],
)

makani_c_library(
    name = "xlr_serial",
    srcs = [
        "xlr_api.c",
        "xlr_serial.c",
    ],
    hdrs = [
        "xlr_api.h",
        "xlr_at.h",
        "xlr_serial.h",
    ],
    deps = [
        "//avionics/common:endian",
        "//avionics/common:serial_parse",
        "//common:macros",
    ],
)

# Python/h2py rules.

h2py_library(
    name = "h2py_ads7828_types",
    function_deps = [":ads7828_types"],
    header = "ads7828_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_bq34z100_types",
    function_deps = [":bq34z100_types"],
    header = "bq34z100_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_ina219_types",
    function_deps = [":ina219_types"],
    header = "ina219_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_ltc2309_types",
    function_deps = [":ltc2309_types"],
    header = "ltc2309_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_ltc4151_types",
    function_deps = [":ltc4151_types"],
    header = "ltc4151_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_ltc6804_types",
    function_deps = [":ltc6804_types"],
    header = "ltc6804_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_mcp342x_types",
    function_deps = [":mcp342x_types"],
    header = "mcp342x_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_mcp9800_types",
    function_deps = [":mcp9800_types"],
    header = "mcp9800_types.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_si7021_types",
    function_deps = [":si7021_types"],
    header = "si7021_types.h",
    visibility = ["//visibility:public"],
)

# Pack2 libraries.

pack2(
    name = "encoder_params",
    src = "encoder_params.p2",
)

pack2(
    name = "hsc_model",
    src = "hsc_model.p2",
)

pack2(
    name = "microhard_params",
    src = "microhard_params.p2",
)

# Code generation rules.

py_binary(
    name = "generate_adis16488_coef",
    srcs = ["generate_adis16488_coef.py"],
    deps = [
        requirement("python-gflags"),
    ],
)

genrule(
    name = "adis16488_coef_genrule",
    srcs = ["adis16488_coef.csv"],
    outs = [
        "adis16488_coef.c",
        "adis16488_coef.h",
    ],
    cmd = " ".join([
        "$(location :generate_adis16488_coef)",
        "--coef_file=$(location adis16488_coef.csv)",
        "--header_file=$(location adis16488_coef.h)",
        "--source_file=$(location adis16488_coef.c)",
    ]),
    tools = [":generate_adis16488_coef"],
)

# Test functions.

makani_c_library(
    name = "adis16488_test",
    srcs = ["adis16488_test.c"],
    hdrs = ["adis16488_test.h"],
    deps = [
        ":adis16488",
        "//avionics/firmware/test",
        "//common/c_math:common",
    ],
)

makani_c_library(
    name = "spi_flash_test",
    srcs = ["spi_flash_test.c"],
    hdrs = ["spi_flash_test.h"],
    deps = [
        ":spi_flash",
        "//avionics/firmware/test",
    ],
)

# Unit tests.

makani_cc_test(
    name = "xlr_api_test",
    srcs = [
        "xlr_api_test.cc",
    ],
    deps = [
        ":xlr_serial",
        "//common:macros",
    ],
)

makani_cc_test(
    name = "xlr_serial_test",
    srcs = [
        "xlr_serial_test.cc",
    ],
    deps = [
        ":xlr_serial",
        "//common:macros",
    ],
)
