package(default_visibility = ["//avionics/motor:__subpackages__"])

load("//avionics/firmware:build_rules.bzl", "tms570_application", "tms570_test")
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")
load("//lib/bazel:autogen_rules.bzl", "h2py_library", "message_ruleset")
load("//lib/bazel:c_rules.bzl", "makani_c_library", "makani_cc_test")
load("//lib/python/pack2:build_rules.bzl", "pack2")

exports_files(["io.c"])

# C Libraries.

makani_c_library(
    name = "current_limit_types",
    hdrs = ["current_limit_types.h"],
)

makani_c_library(
    name = "flags",
    hdrs = ["flags.h"],
    visibility = ["//visibility:public"],
)

# TODO: Make motor code more modular.
# TODO: Get rid of the glob.
makani_c_library(
    name = "motor_lib",
    srcs = glob(
        ["*.c"],
        exclude = [
            "io_cvt.c",
            "io_cvt_dyno.c",
            "main.c",
            "mon_test.c",
            "params.c",
            "test.c",
        ],
    ),
    hdrs = glob(
        ["*.h"],
        exclude = [
            "current_limit_types.h",
            "mon_test.h",
            "params.h",
            "params_types.h",
        ],
    ),
    deps = [
        ":calib_params_c",
        ":config_params_c",
        ":current_limit_types",
        ":flags",
        ":params",
        "//avionics/common:controller_arbitration",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:motor_thermal_types",
        "//avionics/common:safety_codes",
        "//avionics/common/fast_math",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:ad7265",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:led",
        "//avionics/firmware/drivers:mcp342x",
        "//avionics/firmware/drivers:mcp9800",
        "//avionics/firmware/drivers:si7021",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:analog",
        "//avionics/firmware/monitors:mcp342x",
        "//avionics/firmware/monitors:mcp9800",
        "//avionics/firmware/monitors:short_stack_types",
        "//avionics/firmware/monitors:si7021",
        "//avionics/firmware/monitors:thermal_types",
        "//avionics/firmware/network:common",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/params:flash",
        "//avionics/firmware/params:params_server",
        "//avionics/firmware/serial",
        "//avionics/firmware/util:selftest",
        "//avionics/motor/monitors",
        "//avionics/network:message_stats",
        "//common:barrier",
        "//common:macros",
    ],
)

message_ruleset(
    name = "logging",
    generate_cvt_lib = False,
    header = "logging.h",
    visibility = ["//visibility:public"],
    deps = [
        "//avionics/common:motor_foc_types",
    ],
)

makani_c_library(
    name = "param_types",
    hdrs = ["param_types.h"],
)

makani_c_library(
    name = "params",
    srcs = ["params.c"],
    hdrs = ["params.h"],
    deps = [
        ":config_params_c",
        ":param_types",
        "//common:macros",
    ],
)

# C/C++ tests.

makani_cc_test(
    name = "flags_test",
    srcs = ["flags_test.cc"],
    deps = [
        ":flags",
        "//avionics/common:motor_thermal_types",
    ],
)

# Binaries.

# TODO: Make motor code more modular.
tms570_application(
    name = "motor_application.elf",
    srcs = [
        "io_cvt.c",
        "main.c",
    ],
    deps = [
        ":config_params_c",
        ":flags",
        ":motor_lib",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/network:common",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_motor",
    ],
)

tms570_application(
    name = "dyno_motor_application.elf",
    srcs = [
        "io_cvt_dyno.c",
        "main.c",
    ],
    deps = [
        ":calib_params_c",
        ":config_params_c",
        ":flags",
        ":motor_lib",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/network:common",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_dyno_motor",
    ],
)

tms570_test(
    name = "motor_test_application.elf",
    srcs = [
        "io_cvt.c",
        "mon_test.c",
        "mon_test.h",
        "test.c",
    ],
    deps = [
        ":motor_lib",
        "//avionics/firmware/cpu:adc_test",
        "//avionics/firmware/cpu:memcpy_test",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/drivers:bcm",
        "//avionics/firmware/identity",
        "//avionics/network:cvt_motor",
    ],
)

# Python/h2py rules.

h2py_library(
    name = "h2py_flags",
    header = "flags.h",
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_params",
    function_deps = [":params"],
    header = "params.h",
    visibility = ["//visibility:public"],
)

# Pack2 libraries.

pack2(
    name = "config_params",
    src = "config_params.p2",
)

pack2(
    name = "calib_params",
    src = "calib_params.p2",
)

# Parameter files.

[
    gen_param_file(
        src = "config_params.yaml",
        out = key + "_motor_config_params.bin",
        key = key,
    )
    for key in [
        "calibrate_protean_swap_ac",
        "calibrate_protean_swap_none",
        "calibrate_yasa_swap_ac",
        "calibrate_yasa_swap_none",
        "protean_swap_ac_prop_rev1_negative_single",
        "protean_swap_ac_prop_rev1_negative_stacked",
        "yasa_swap_ac_prop_rev1_negative_single",
        "yasa_swap_ac_prop_rev2_positive_single",
        "yasa_swap_none_prop_rev1_negative_single",
        "yasa_swap_none_prop_rev1_negative_stacked",
        "yasa_swap_none_prop_rev2_positive_single",
        "yasa_swap_none_prop_rev2_positive_stacked",
    ]
]

[
    gen_param_file(
        src = "config_params.yaml",
        out = key + "_dyno_motor_config_params.bin",
        key = key,
    )
    for key in [
        "yasa_swap_ac_dyno_single",
        "yasa_swap_none_dyno_single",
    ]
]

[
    gen_param_file(
        src = "calib_params.yaml",
        out = key + "_motor_calib_params.bin",
        key = key,
    )
    for key in [
        "iron_bird_pbi",
        "iron_bird_pbo",
        "iron_bird_pti",
        "iron_bird_pto",
        "iron_bird_sbi",
        "iron_bird_sbo",
        "iron_bird_sti",
        "iron_bird_sto",
        "lollipop",
        "m600a_sn001_pbi",
        "m600a_sn001_pbo",
        "m600a_sn001_pti",
        "m600a_sn001_pto",
        "m600a_sn001_sbi",
        "m600a_sn001_sbo",
        "m600a_sn001_sti",
        "m600a_sn001_sto",
        "m600a_sn002_pbi",
        "m600a_sn002_pbo",
        "m600a_sn002_pti",
        "m600a_sn002_pto",
        "m600a_sn002_sbi",
        "m600a_sn002_sbo",
        "m600a_sn002_sti",
        "m600a_sn002_sto",
        "m600a_sn003_pbi",
        "m600a_sn003_pbo",
        "m600a_sn003_pti",
        "m600a_sn003_pto",
        "m600a_sn003_sbi",
        "m600a_sn003_sbo",
        "m600a_sn003_sti",
        "m600a_sn003_sto",
        "m600a_sn004_pbi",
        "m600a_sn004_pbo",
        "m600a_sn004_pti",
        "m600a_sn004_pto",
        "m600a_sn004_sbi",
        "m600a_sn004_sbo",
        "m600a_sn004_sti",
        "m600a_sn004_sto",
        "m600a_sn005_pbi",
        "m600a_sn005_pbo",
        "m600a_sn005_pti",
        "m600a_sn005_pto",
        "m600a_sn005_sbi",
        "m600a_sn005_sbo",
        "m600a_sn005_sti",
        "m600a_sn005_sto",
        "m600a_sn006_pbi",
        "m600a_sn006_pbo",
        "m600a_sn006_pti",
        "m600a_sn006_pto",
        "m600a_sn006_sbi",
        "m600a_sn006_sbo",
        "m600a_sn006_sti",
        "m600a_sn006_sto",
        "null_calibration",
    ]
]

[
    gen_param_file(
        src = "calib_params.yaml",
        out = key + "_dyno_motor_calib_params.bin",
        key = key,
    )
    for key in [
        "1x1_dyno_yasa_sn1",
        "1x1_dyno_yasa_sn7",
        "8x8_dyno_yasa_pbi",
        "8x8_dyno_yasa_pbo",
        "8x8_dyno_yasa_pti",
        "8x8_dyno_yasa_pto",
        "8x8_dyno_yasa_sbi",
        "8x8_dyno_yasa_sbo",
        "8x8_dyno_yasa_sti",
        "8x8_dyno_yasa_sto",
        "null_calibration",
    ]
]
