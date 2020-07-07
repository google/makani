package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load(
    "//lib/bazel:autogen_rules.bzl",
    "h2py_library",
    "message_ruleset",
)
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_binary",
    "makani_cc_library",
    "makani_cc_test",
)
load("//lib/bazel:py_rules.bzl", "py_binary")

message_ruleset(
    name = "sim_messages",
    header = "sim_messages.h",
    source = "sim_messages.c",
    deps = [
        "//common/c_math",
        "//control:control_telemetry",
        "//control:control_types",
        "//control:system_types",
        "//control/avionics:avionics_interface_types",
        "//control/estimator:estimator_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "sim_types",
    hdrs = ["sim_types.h"],
    deps = [
        "//avionics/common:gps_receiver_c",
        "//common/c_math",
        "//control:sensor_types",
        "//control:system_types",
        "//sim/physics:aero_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "sim_params",
    srcs = ["sim_params.c"],
    hdrs = ["sim_params.h"],
    deps = [
        ":sim_types",
    ],
)

genrule(
    name = "sim_params_genrule",
    srcs = ["//config:common/sim/sim_params.py"],
    outs = ["sim_params.c"],
    cmd = " ".join([
        "$(location //config:write_params_light)",
        "--type=sim",
        "--input_file=$(location //config:common/sim/sim_params.py)",
        "--output_file=$@",
    ]),
    tools = ["//config:write_params_light"],
)

h2py_library(
    name = "h2py_sim_types",
    function_deps = ["//common/c_math:c_math"],
    header = "sim_types.h",
    header_deps = [":sim_types"],
)

message_ruleset(
    name = "sim_telemetry",
    header = "sim_telemetry.h",
    source = "sim_telemetry.c",
    deps = [
        ":sim_types",
        "//common/c_math",
        "//control:actuator_types",
        "//control:sensor_types",
        "//control:system_types",
        "//system:labels",
    ],
)

makani_cc_library(
    name = "interfaces",
    hdrs = ["interfaces.h"],
)

makani_cc_library(
    name = "state",
    srcs = ["state.cc"],
    hdrs = ["state.h"],
    deps = [
        ":interfaces",
        "//common/c_math",
        "//control:system_types",
        "//lib/util:base64",
        "@usr_lib//:glog",
    ],
)

makani_cc_library(
    name = "faults",
    srcs = ["faults/faults.cc"],
    hdrs = ["faults/faults.h"],
    deps = [
        ":sim_types",
        "//common:macros",
        "@usr_lib//:glog",
    ],
)

makani_cc_library(
    name = "sim_library",
    srcs = [
        "json_formatter.cc",
        "json_formatter.h",
        "sim_base.cc",
        "sim_base.h",
        "sim_comms.cc",
        "sim_comms.h",
        "sim_dynamic_system.cc",
        "sim_dynamic_system.h",
        "sim_save_states.cc",
        "sim_save_states.h",
    ],
    deps = [
        ":cvt_sim_messages",
        ":faults",
        ":interfaces",
        ":pack_sim_telemetry",
        ":sim_params",
        ":sim_types",
        "//avionics/common:controller_arbitration",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:pack_tether_message",
        "//avionics/common:tether_convert",
        "//avionics/common:tether_message",
        "//avionics/common:tether_message_types",
        "//avionics/linux:aio",
        "//avionics/linux:clock",
        "//avionics/linux:cvt_util",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:backtrace",
        "//common:macros",
        "//common:runfiles_dir",
        "//control:common",
        "//control:control_util",
        "//control:cvt_control_telemetry",
        "//control:system_params",
        "//control:system_types",
        "//lib/json_load:json_load_or_die",
        "//lib/json_load:load_params",
        "//sim/math",
        "//sim/models:full_system",
        "//system:labels",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_binary(
    name = "sim",
    srcs = [
        "kite_sim.cc",
        "kite_sim.h",
    ],
    deps = [":sim_library"],
)

makani_cc_binary(
    name = "tether_sim",
    srcs = [
        "tether_sim.cc",
        "tether_sim.h",
    ],
    deps = [
        ":sim_library",
        "//lib/pcap_to_hdf5:message_log",
        "//sim/models:tether_simulator_system",
    ],
)

py_binary(
    name = "run_sim",
    srcs = ["run_sim.py"],
    data = [
        ":sim",
        "//config:write_params",
        "//control:hitl_controller",
        "//control:hitl_ground_estimator",
        "//control:sim_controller",
        "//control:sim_ground_estimator",
        "//gs/aio_snapshot",
        "//gs/monitor",
        "//gs/monitor2:webmonitor_launcher",
        "//lib/flight_gear:flight_gear_interface",
        "//lib/joystick",
        "//lib/pcap_to_hdf5",
        "//lib/scripts:sim_tcpdump",
        "//vis",
    ],
    deps = [
        "//lib/bazel:bazel_util",
        "//lib/python:dict_util",
        "//lib/python:os_util",
        "//lib/python:turbsim_util",
        "//lib/python:wing_flag",
        requirement("python-gflags"),
    ],
)

makani_cc_test(
    name = "faults_test",
    srcs = ["faults/faults_test.cc"],
    deps = [
        ":faults",
        "//lib/util:test_util",
    ],
)
