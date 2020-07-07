package(default_visibility = ["//visibility:public"])

load("//lib/bazel:c_rules.bzl", "makani_c_library", "makani_cc_test")

makani_c_library(
    name = "avionics_interface_types",
    srcs = ["avionics_interface_types.c"],
    hdrs = ["avionics_interface_types.h"],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/common:loadcell_types",
        "//avionics/common:tether_message_types",
        "//avionics/network:aio_labels",
        "//common:macros",
        "//control:system_types",
        "//control/experiments:experiment_types",
        "//control/fault_detection:fault_detection_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "avionics_interface",
    srcs = [
        "avionics_conversion.c",
        "avionics_conversion.h",
        "avionics_faults.c",
        "avionics_faults.h",
        "avionics_interface.c",
        "avionics_saturation.c",
        "avionics_saturation.h",
    ],
    hdrs = ["avionics_interface.h"],
    nostatic_files = [
        "avionics_faults.c",
        "avionics_saturation.c",
    ],
    deps = [
        ":avionics_interface_types",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:encoder_types",
        "//avionics/common:pack_tether_message",
        "//avionics/common:safety_codes",
        "//avionics/common:servo_types",
        "//avionics/common:tether_convert",
        "//avionics/common:tether_message",
        "//avionics/common:winch_messages",
        "//avionics/motor/firmware:flags",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_all",
        "//common:macros",
        "//common/c_math",
        "//control:common",
        "//control:control_types",
        "//control:sensor_types",
        "//control:system_types",
        "//control/fault_detection",
        "//system:labels",
    ],
)

makani_c_library(
    name = "avionics_sim",
    srcs = ["avionics_sim.c"],
    hdrs = ["avionics_sim.h"],
    nostatic_files = ["avionics_sim.c"],
    deps = [
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:pack_tether_message",
        "//avionics/common:tether_convert",
        "//avionics/linux:aio",
        "//avionics/network:aio_labels",
        "//avionics/network:message_type",
        "//control:system_types",
        "//sim:sim_messages",
        "//system:labels",
    ],
)

makani_cc_test(
    name = "avionics_interface_test",
    srcs = ["avionics_interface_test.cc"],
    deps = [
        "avionics_interface",
        "//avionics/common:cvt_avionics_messages",
        "//control:control_params",
        "//control:system_params",
        "//control/fault_detection:fault_detection_types",
        "//system:labels",
        "@usr_lib//:glog",
    ],
)

makani_cc_test(
    name = "avionics_faults_test",
    srcs = ["avionics_faults_test.cc"],
    deps = [
        ":avionics_interface_nostatic",
        "//common:macros",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "avionics_saturation_test",
    srcs = ["avionics_saturation_test.cc"],
    deps = [
        "avionics_interface_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "avionics_sim_test",
    srcs = ["avionics_sim_test.cc"],
    deps = [
        ":avionics_interface",
        ":avionics_sim_nostatic",
        "//avionics/common:cvt_avionics_messages",
        "//lib/util:test_util",
    ],
)
