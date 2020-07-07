package(default_visibility = ["//visibility:public"])

load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load("//lib/bazel:c_rules.bzl", "makani_c_library")

makani_c_library(
    name = "fault_detection_types",
    srcs = ["fault_detection_types.c"],
    hdrs = ["fault_detection_types.h"],
    deps = [
        "//avionics/network:aio_labels",
        "//control:sensor_types",
        "//system:labels",
    ],
)

h2py_library(
    name = "h2py_fault_detection_types",
    function_deps = [":fault_detection_types"],
    header = "fault_detection_types.h",
)

# TODO: Prune dependencies.  This target doesn't actually depend on
# much of this anymore. (Most of its functionality moved to avionics_faults.)
makani_c_library(
    name = "fault_detection",
    srcs = [
        "fault_detection_disabled.c",
        "fault_detection_util.c",
        "fault_detection_util.h",
    ],
    hdrs = [
        "fault_detection_disabled.h",
        "fault_detection_util.h",
    ],
    deps = [
        ":fault_detection_types",
        "//avionics/common:avionics_messages",
        "//avionics/common:novatel_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_util",
        "//control:sensor_types",
        "//control:system_types",
        "//control/avionics:avionics_interface_types",
        "//control/estimator:estimator_types",
        "//sim:sim_messages",
        "//system:labels",
    ],
)
