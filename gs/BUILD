package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_cc_binary",
    "makani_cc_library",
    "makani_cc_test",
)

makani_cc_binary(
    name = "batt_command",
    srcs = ["batt_command/batt_command.cc"],
    deps = [
        "//avionics/common:batt_types",
        "//avionics/common:safety_codes",
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//lib/util:operator_confirmation",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_binary(
    name = "flight_command",
    srcs = ["flight_command/flight_command.cc"],
    deps = [
        ":flight_commander",
        "//avionics/linux:aio",
        "//control:control_params",
        "//control/experiments:experiment_types",
        "//lib/util:operator_confirmation",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_library(
    name = "flight_commander",
    srcs = ["flight_command/flight_commander.cc"],
    hdrs = ["flight_command/flight_commander.h"],
    deps = [
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:safety_codes",
        "//avionics/common:tether_message",
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//control:control_types",
        "//control/experiments:experiment_util",
        "//control/hover:hover_types",
        "//sim:cvt_sim_messages",
        "@usr_lib//:glog",
    ],
)

makani_cc_test(
    name = "flight_commander_test",
    srcs = ["flight_command/flight_commander_test.cc"],
    tags = ["exclusive"],
    deps = [
        ":flight_commander",
        "//avionics/linux:cvt_util",
    ],
)

makani_cc_binary(
    name = "fpv_command",
    srcs = ["fpv_command/fpv_command.cc"],
    deps = [
        "//avionics/common:safety_codes",
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//lib/util:operator_confirmation",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_binary(
    name = "loadcell_command",
    srcs = ["loadcell_command/loadcell_command.cc"],
    deps = [
        "//avionics/common:cvt",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:endian",
        "//avionics/common:network_config",
        "//avionics/common:pack_aio_header",
        "//avionics/linux:aio",
        "//avionics/linux:clock",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//lib/util:operator_confirmation",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_binary(
    name = "mvlv_command",
    srcs = ["mvlv_command/mvlv_command.cc"],
    deps = [
        "//avionics/common:mvlv_types",
        "//avionics/common:safety_codes",
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//lib/util:operator_confirmation",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_binary(
    name = "pitot_command",
    srcs = ["pitot_command/pitot_command.cc"],
    deps = [
        "//avionics/common:safety_codes",
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//lib/util:operator_confirmation",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_binary(
    name = "short_stack_command",
    srcs = ["short_stack_command/short_stack_command.cc"],
    deps = [
        "//avionics/common:safety_codes",
        "//avionics/common:short_stack_types",
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//lib/util:operator_confirmation",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_binary(
    name = "state_command",
    srcs = [
        "state_command/detwist_state.cc",
        "state_command/detwist_state.h",
        "state_command/gs02_state.cc",
        "state_command/gs02_state.h",
        "state_command/main.cc",
        "state_command/motor_state.cc",
        "state_command/motor_state.h",
        "state_command/servo_state.cc",
        "state_command/servo_state.h",
        "state_command/state_command.cc",
        "state_command/state_command.h",
        "state_command/tether_release_state.cc",
        "state_command/tether_release_state.h",
        "state_command/winch_state.cc",
        "state_command/winch_state.h",
    ],
    deps = [
        "//avionics/common:actuator_types",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:safety_codes",
        "//avionics/common:tether_message",
        "//avionics/linux:aio",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//lib/util:operator_confirmation",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)
