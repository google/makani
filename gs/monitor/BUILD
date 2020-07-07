package(default_visibility = ["//visibility:public"])

load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_binary",
    "makani_cc_test",
)

genrule(
    name = "monitor_params_genrule",
    srcs = ["//config:common/monitor/monitor_params.py"],
    outs = ["monitor_params.c"],
    cmd = " ".join([
        "$(location //config:write_params)",
        "--type=monitor",
        "--input_file=$(location //config:common/monitor/monitor_params.py)",
        "--output_file=$@",
    ]),
    tools = ["//config:write_params"],
)

makani_c_library(
    name = "monitor_common",
    srcs = [
        "common.c",
        "common.h",
    ],
    hdrs = ["common.h"],
    deps = [
        "//control:common",
        "//control:control_types",
        "//control/crosswind:crosswind_types",
        "//control/hover:hover_types",
        "//control/trans_in:trans_in_types",
    ],
)

h2py_library(
    name = "h2py_monitor_common",
    function_deps = [":monitor_common"],
    header = "common.h",
)

makani_c_library(
    name = "monitor_types",
    hdrs = ["monitor_types.h"],
    deps = ["//common/c_math"],
)

makani_c_library(
    name = "monitor_params",
    srcs = ["monitor_params.c"],
    hdrs = ["monitor_params.h"],
    deps = [":monitor_types"],
)

h2py_library(
    name = "h2py_monitor_types",
    header = "monitor_types.h",
    header_deps = [":monitor_types"],
)

h2py_library(
    name = "h2py_monitor_params",
    function_deps = [":monitor_params"],
    header = "monitor_params.h",
)

makani_c_library(
    name = "monitor_lib",
    srcs = glob(
        [
            "**/*.h",
            "**/*.c",
        ],
        exclude = [
            "monitor_types.h",
            "**/*_test.cc",
        ],
    ),
    copts = ["-pthread"],
    defines = ["_POSIX_C_SOURCE=200809L"],
    deps = [
        ":monitor_common",
        ":monitor_params",
        ":monitor_types",
        ":system_build",
        "//avionics/common:build_info",
        "//avionics/common:encoder_types",
        "//avionics/common:gps_receiver_c",
        "//avionics/common:motor_thermal_types",
        "//avionics/common:network_config",
        "//avionics/common:winch_messages",
        "//avionics/firmware/drivers:mcp9800_types",
        "//avionics/firmware/drivers:si7021_types",
        "//avionics/motor/firmware:flags",
        "//avionics/network:routes",
        "//avionics/network:switch_links",
        "//common:macros",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_types",
        "//control:control_util",
        "//control:pack_control_telemetry",
        "//control:system_params",
        "//control:system_types",
        "//control/crosswind",
        "//control/crosswind:crosswind_types",
        "//control/estimator",
        "//control/estimator:estimator_types",
        "//control/fault_detection:fault_detection_types",
        "//control/hover",
        "//control/trans_in",
        "//gs/aio_snapshot:pack_aio_telemetry",
        "//lib/udpio",
        "//system:labels",
        "@usr_include//:gtk",
    ],
)

makani_cc_binary(
    name = "monitor",
    srcs = ["monitor.cc"],
    deps = [
        ":monitor_lib",
        "//lib/json_load:load_params",
    ],
)

makani_c_library(
    name = "system_build",
    srcs = ["indicators/system_build.c"],
    hdrs = ["indicators/system_build.h"],
    deps = [
        "//avionics/common:build_info_types",
        "//avionics/network:aio_node",
        "//common:macros",
    ],
)

# Unit tests.

makani_cc_test(
    name = "system_build_test",
    srcs = ["indicators/system_build_test.cc"],
    deps = [
        ":system_build",
    ],
)
