package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:autogen_rules.bzl",
    "h2py_library",
    "message_ruleset",
)
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_binary",
    "makani_c_library",
    "makani_cc_binary",
    "makani_cc_test",
)

makani_c_library(
    name = "sensor_types",
    hdrs = ["sensor_types.h"],
    deps = [
        "//avionics/common:novatel_types",
        "//avionics/common:plc_messages",
        "//common/c_math",
        "//system:labels",
    ],
)

h2py_library(
    name = "h2py_sensor_types",
    header = "sensor_types.h",
    header_deps = [":sensor_types"],
)

makani_c_library(
    name = "system_types",
    srcs = ["system_types.c"],
    hdrs = ["system_types.h"],
    deps = [
        "//avionics/common:novatel_types",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//common/c_math",
        "//system:labels",
    ],
)

makani_c_library(
    name = "system_params",
    srcs = ["system_params.c"],
    hdrs = ["system_params.h"],
    deps = [
        ":system_types",
    ],
)

genrule(
    name = "system_params_genrule",
    srcs = ["//config:m600/system_params.py"],
    outs = ["system_params.c"],
    cmd = " ".join([
        "$(location //config:write_params_light)",
        "--type=system",
        "--input_file=$(location //config:m600/system_params.py)",
        "--output_file=$@",
    ]),
    tools = ["//config:write_params_light"],
)

h2py_library(
    name = "h2py_system_types",
    function_deps = [":system_types"],
    header = "system_types.h",
)

h2py_library(
    name = "h2py_system_params",
    function_deps = [
        ":system_params",
    ],
    header = "system_params.h",
)

makani_c_library(
    name = "control_types",
    srcs = ["control_types.c"],
    hdrs = ["control_types.h"],
    deps = [
        ":system_types",
        "//common/c_math",
        "//control/avionics:avionics_interface_types",
        "//control/crosswind:crosswind_types",
        "//control/estimator:estimator_types",
        "//control/fault_detection:fault_detection_types",
        "//control/hover:hover_types",
        "//control/manual:manual_types",
        "//control/trans_in:trans_in_types",
    ],
)

makani_c_library(
    name = "control_params",
    srcs = ["control_params.c"],
    hdrs = ["control_params.h"],
    deps = [
        ":control_types",
    ],
)

genrule(
    name = "control_params_genrule",
    srcs = ["//config:common/control/control_params.py"],
    outs = ["control_params.c"],
    cmd = " ".join([
        "$(location //config:write_params)",
        "--type=control",
        "--input_file=$(location //config:common/control/control_params.py)",
        "--output_file=$@",
    ]),
    tools = ["//config:write_params"],
)

h2py_library(
    name = "h2py_control_params",
    function_deps = [
        ":control_params",
    ],
    header = "control_params.h",
)

h2py_library(
    name = "h2py_control_types",
    header = "control_types.h",
    header_deps = [":control_types"],
)

message_ruleset(
    name = "control_telemetry",
    header = "control_telemetry.h",
    source = "control_telemetry.c",
    deps = [
        ":actuator_types",
        ":common",
        ":control_types",
        ":sensor_types",
        ":system_types",
        "//avionics/common:avionics_messages",
        "//avionics/common:tether_message_types",
        "//avionics/linux:q7_slow_status_types",
        "//common/c_math",
        "//control/avionics:avionics_interface_types",
        "//control/estimator:estimator_types",
        "//control/fault_detection:fault_detection_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "common",
    srcs = ["common.c"],
    hdrs = ["common.h"],
    deps = [":control_types"],
)

h2py_library(
    name = "h2py_common",
    function_deps = [
        ":common",
    ],
    header = "common.h",
)

makani_c_library(
    name = "simple_aero_types",
    hdrs = ["simple_aero_types.h"],
    deps = ["//system:labels"],
)

makani_c_library(
    name = "simple_aero",
    srcs = ["simple_aero.c"],
    hdrs = ["simple_aero.h"],
    nostatic_files = [
        "simple_aero.c",
    ],
    deps = [
        ":simple_aero_types",
        "//common:macros",
        "//common/c_math",
    ],
)

h2py_library(
    name = "h2py_simple_aero",
    function_deps = [":simple_aero"],
    header = "simple_aero.h",
)

makani_c_library(
    name = "ground_frame",
    srcs = ["ground_frame.c"],
    hdrs = ["ground_frame.h"],
    deps = [
        ":system_params",
        "//common/c_math",
    ],
)

makani_c_library(
    name = "vessel_frame",
    srcs = ["vessel_frame.c"],
    hdrs = ["vessel_frame.h"],
    deps = [
        ":system_params",
        "//common/c_math",
    ],
)

makani_cc_test(
    name = "vessel_frame_test",
    srcs = ["vessel_frame_test.cc"],
    deps = [
        ":control_util",
        ":vessel_frame",
    ],
)

makani_c_library(
    name = "control_util",
    srcs = [
        "perch_frame.c",
        "platform_frame.c",
        "sensor_util.c",
        "tether_util.c",
    ],
    hdrs = [
        "perch_frame.h",
        "platform_frame.h",
        "sensor_util.h",
        "tether_util.h",
    ],
    nostatic_files = [
        "sensor_util.c",
    ],
    deps = [
        ":actuator_util",
        ":ground_frame",
        ":sensor_types",
        ":system_params",
        ":vessel_frame",
        "//common:macros",
        "//common/c_math",
        "//system:labels",
    ],
)

makani_c_library(
    name = "actuator_types",
    hdrs = ["actuator_types.h"],
    deps = [
        ":simple_aero_types",
        "//avionics/network:aio_labels",
        "//common/c_math",
    ],
)

makani_c_library(
    name = "actuator_util",
    srcs = ["actuator_util.c"],
    hdrs = ["actuator_util.h"],
    nostatic_files = [
        "actuator_util.c",
    ],
    deps = [
        ":actuator_types",
        ":control_types",
        ":ground_frame",
        ":simple_aero",
        ":system_types",
        "//common:macros",
        "//common/c_math",
        "//system:labels",
    ],
)

h2py_library(
    name = "h2py_actuator_util",
    function_deps = [":actuator_util"],
    header = "actuator_util.h",
)

h2py_library(
    name = "h2py_sensor_util",
    function_deps = [
        ":control_util",
    ],
    header = "sensor_util.h",
)

h2py_library(
    name = "h2py_ground_frame",
    function_deps = [":ground_frame"],
    header = "ground_frame.h",
)

makani_cc_test(
    name = "ground_frame_test",
    srcs = ["ground_frame_test.cc"],
    deps = [
        ":control_util",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "perch_frame_test",
    srcs = ["perch_frame_test.cc"],
    deps = [
        ":control_util",
        "//lib/util:test_util",
    ],
)

makani_c_library(
    name = "control_output",
    srcs = ["control_output.c"],
    hdrs = ["control_output.h"],
    deps = [
        ":common",
        ":control_params",
        ":control_types",
        ":control_util",
        "//common/c_math",
        "//system:labels",
    ],
)

makani_c_library(
    name = "control_planner",
    srcs = ["control_planner.c"],
    hdrs = ["control_planner.h"],
    nostatic_files = ["control_planner.c"],
    deps = [
        ":common",
        ":control_params",
        ":control_telemetry",
        ":control_types",
        ":control_util",
        ":system_types",
        "//common/c_math",
        "//system:labels",
    ],
)

makani_c_library(
    name = "control_system",
    srcs = ["control_system.c"],
    hdrs = ["control_system.h"],
    deps = [
        ":common",
        ":control_output",
        ":control_planner",
        ":control_telemetry",
        ":control_types",
        ":control_util",
        ":system_types",
        "//avionics/common:avionics_messages",
        "//avionics/common:build_info",
        "//avionics/linux:clock",
        "//common/c_math",
        "//control/avionics:avionics_interface",
        "//control/crosswind",
        "//control/estimator",
        "//control/fault_detection",
        "//control/hover",
        "//control/manual",
        "//control/trans_in",
        "//system:labels",
    ],
)

makani_c_binary(
    name = "flight_controller",
    srcs = ["control_aio_main.c"],
    archs = [
        "q7",
        "x86",
    ],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/common:pack_avionics_messages",
        "//avionics/linux:aio",
        "//avionics/linux:q7_slow_status",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
        "//control:control_system",
        "//control:control_types",
        "//control:pack_control_telemetry",
        "//control:system_types",
    ],
)

makani_c_binary(
    name = "ground_estimator",
    srcs = ["ground_aio_main.c"],
    archs = [
        "q7",
        "x86",
    ],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/common:pack_avionics_messages",
        "//avionics/linux:aio",
        "//avionics/linux:q7_slow_status",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
        "//control:control_system",
        "//control:control_types",
        "//control:pack_control_telemetry",
        "//control:system_types",
        "//control/estimator:estimator_ground",
    ],
)

message_ruleset(
    name = "ground_telemetry",
    header = "ground_telemetry.h",
    source = "ground_telemetry.c",
    deps = [
        ":control_types",
    ],
)

makani_c_binary(
    name = "hitl_controller",
    srcs = ["control_aio_main.c"],
    archs = [
        "q7",
        "x86",
    ],
    defines = ["MAKANI_HITL"],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/common:pack_avionics_messages",
        "//avionics/linux:aio",
        "//avionics/linux:q7_slow_status",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
        "//control:control_params",
        "//control:control_system",
        "//control:control_types",
        "//control:pack_control_telemetry",
        "//control:system_params",
        "//control:system_types",
        "//control/avionics:avionics_sim",
        "//sim:cvt_sim_messages",
    ],
)

makani_cc_binary(
    name = "sim_controller",
    srcs = [
        "control_main.cc",
        "control_main.h",
    ],
    deps = [
        ":control_params",
        ":control_system",
        ":control_types",
        ":pack_control_telemetry",
        ":system_params",
        ":system_types",
        "//avionics/common:pack_avionics_messages",
        "//avionics/linux:aio",
        "//avionics/linux:cvt_util",
        "//avionics/linux:q7_slow_status",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:backtrace",
        "//common:macros",
        "//control/avionics:avionics_sim",
        "//lib/json_load:load_params",
        "//sim:cvt_sim_messages",
        "//system:labels",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_c_binary(
    name = "hitl_ground_estimator",
    srcs = ["ground_aio_main.c"],
    archs = [
        "q7",
        "x86",
    ],
    defines = ["MAKANI_HITL"],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/common:pack_avionics_messages",
        "//avionics/linux:aio",
        "//avionics/linux:q7_slow_status",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
        "//control:control_params",
        "//control:control_system",
        "//control:control_types",
        "//control:pack_control_telemetry",
        "//control:system_params",
        "//control:system_types",
        "//control/avionics:avionics_sim",
        "//control/estimator:estimator_ground",
        "//sim:cvt_sim_messages",
    ],
)

makani_cc_binary(
    name = "sim_ground_estimator",
    srcs = [
        "ground_main.cc",
        "ground_main.h",
    ],
    deps = [
        ":control_params",
        ":control_system",
        ":control_types",
        ":pack_control_telemetry",
        ":system_params",
        ":system_types",
        "//avionics/common:pack_avionics_messages",
        "//avionics/linux:aio",
        "//avionics/linux:cvt_util",
        "//avionics/linux:q7_slow_status",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:backtrace",
        "//common:macros",
        "//control/avionics:avionics_sim",
        "//control/estimator:estimator_ground",
        "//lib/json_load:load_params",
        "//sim:cvt_sim_messages",
        "//system:labels",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_test(
    name = "actuator_util_test",
    srcs = ["actuator_util_test.cc"],
    deps = [
        ":actuator_util_nostatic",
        ":control_params",
        ":system_params",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "control_planner_test",
    srcs = ["control_planner_test.cc"],
    deps = [
        ":control_planner_nostatic",
        ":control_telemetry",
        "//control/fault_detection",
    ],
)

makani_cc_test(
    name = "sensor_util_test",
    srcs = ["sensor_util_test.cc"],
    deps = [
        ":control_util_nostatic",
        ":system_params",
        "//lib/util:test_util",
        "//sim/models:loadcell",
    ],
)

makani_cc_test(
    name = "simple_aero_test",
    srcs = ["simple_aero_test.cc"],
    deps = [
        ":control_params",
        ":simple_aero_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "tether_util_test",
    srcs = ["tether_util_test.cc"],
    deps = [
        ":control_params",
        ":control_util",
        "//control:control_types",
        "//lib/util:test_util",
    ],
)
