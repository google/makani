package(default_visibility = ["//visibility:private"])

load("@python_pip//:requirements.bzl", "requirement")
load("//avionics/firmware:build_rules.bzl", "tms570_application")
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")
load(
    "//lib/bazel:autogen_rules.bzl",
    "h2py_library",
)
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/bazel:py_rules.bzl", "py_binary")
load("//lib/python/pack2:build_rules.bzl", "pack2")

exports_files(["config_params.yaml"])

makani_c_library(
    name = "r22_types",
    hdrs = ["r22_types.h"],
)

py_binary(
    name = "generate_r22_param",
    srcs = ["generate_r22_param.py"],
    deps = [
        requirement("numpy"),
        requirement("python-gflags"),
        requirement("PyYAML"),
    ],
)

genrule(
    name = "r22_param_genrule",
    srcs = [
        "r22_m600.ccx",
        "r22_param.yaml",
    ],
    outs = [
        "r22_param.c",
        "r22_param.h",
    ],
    cmd = " ".join([
        "$(location :generate_r22_param)",
        "--ccx_file=$(location r22_m600.ccx)",
        "--dict_file=$(location r22_param.yaml)",
        "--source_file=$(location r22_param.c)",
        "--header_file=$(location r22_param.h)",
    ]),
    tools = [":generate_r22_param"],
)

tms570_application(
    name = "servo_application.elf",
    srcs = glob([
        "*.c",
        "*.h",
    ]) + [
        "r22_param.c",
        "r22_param.h",
    ],
    deps = [
        ":calib_params_c",
        ":config_params_c",
        "//avionics/common:actuator_types",
        "//avionics/common:bits",
        "//avionics/common:build_info",
        "//avionics/common:controller_arbitration",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:faults",
        "//avionics/common:network_config",
        "//avionics/common:safety_codes",
        "//avionics/common:servo_types",
        "//avionics/common:tether_message",
        "//avionics/common:tether_message_types",
        "//avionics/common/fast_math",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/cpu:peripherals",
        "//avionics/firmware/drivers:canopen",
        "//avionics/firmware/drivers:ext_watchdog",
        "//avionics/firmware/drivers:led",
        "//avionics/firmware/drivers:log",
        "//avionics/firmware/identity",
        "//avionics/firmware/monitors:aio",
        "//avionics/firmware/monitors:analog",
        "//avionics/firmware/monitors:servo",
        "//avionics/firmware/network:aio",
        "//avionics/firmware/network:net_mon",
        "//avionics/firmware/network:switch_config",
        "//avionics/firmware/output:slow_status",
        "//avionics/firmware/serial",
        "//avionics/firmware/util:selftest",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_servo",
        "//avionics/network:message_type",
        "//common:macros",
        "//common/c_math:common",
    ],
)

h2py_library(
    name = "h2py_r22_param",
    header = "r22_param.h",
    header_deps = [":r22_types"],
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_r22_types",
    header = "r22_types.h",
    visibility = ["//visibility:public"],
)

pack2(
    name = "calib_params",
    src = "calib_params.p2",
)

pack2(
    name = "config_params",
    src = "config_params.p2",
)

[
    gen_param_file(
        src = "calib_params.yaml",
        out = key + "_servo_calib_params.bin",
        key = key,
    )
    for key in [
        "%s_%s" % (flap, system)
        for flap in [
            "a1",
            "a2",
            "a4",
            "a5",
            "a7",
            "a8",
            "e1",
            "e2",
            "r1",
            "r2",
        ]
        for system in [
            "ib",
            "sn1",
            "sn4",
            "sn5",
            "sn6",
            "sn7",
        ]
    ] + [
        "e1_bench",
        "e2_bench",
        "zero_ccw",
        "zero_cw",
    ]
]

[
    gen_param_file(
        src = "config_params.yaml",
        out = key + "_servo_config_params.bin",
        key = key,
    )
    for key in [
        "aileron_a1",
        "aileron_a2",
        "aileron_a2_sn1",
        "aileron_a4",
        "aileron_a5",
        "aileron_a7",
        "aileron_a7_sn1",
        "aileron_a8",
        "elevator",
        "ib_aileron",
        "rudder",
    ]
]
