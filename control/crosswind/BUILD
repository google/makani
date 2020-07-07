package(default_visibility = ["//visibility:public"])

load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load("//lib/bazel:c_rules.bzl", "makani_c_library", "makani_cc_test")

makani_c_library(
    name = "crosswind_types",
    srcs = ["crosswind_types.c"],
    hdrs = ["crosswind_types.h"],
    deps = [
        ":crosswind_playbook_types",
        "//common/c_math",
        "//control:actuator_types",
        "//control:system_types",
        "//control/experiments:crosswind_experiment_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "crosswind_frame",
    srcs = ["crosswind_frame.c"],
    hdrs = ["crosswind_frame.h"],
    deps = [
        "//common/c_math",
    ],
)

makani_cc_test(
    name = "crosswind_frame_test",
    srcs = ["crosswind_frame_test.cc"],
    deps = [
        ":crosswind_frame",
        "//lib/util:test_util",
    ],
)

makani_c_library(
    name = "crosswind",
    srcs = [
        "crosswind.c",
        "crosswind_curvature.c",
        "crosswind_curvature.h",
        "crosswind_inner.c",
        "crosswind_inner.h",
        "crosswind_mode.c",
        "crosswind_mode.h",
        "crosswind_output.c",
        "crosswind_output.h",
        "crosswind_path.c",
        "crosswind_power.c",
        "crosswind_power.h",
        "crosswind_util.c",
        "crosswind_util.h",
    ],
    hdrs = [
        "crosswind.h",
        "crosswind_path.h",
    ],
    nostatic_files = [
        "crosswind_inner.c",
        "crosswind_output.c",
        "crosswind_path.c",
        "crosswind_power.c",
        "crosswind_util.c",
    ],
    deps = [
        ":crosswind_frame",
        ":crosswind_playbook",
        ":crosswind_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control/avionics:avionics_interface_types",
        "//control/estimator:estimator_types",
        "//control/experiments:crosswind_experiment",
        "//control/fault_detection:fault_detection_types",
        "//control/hover:hover_types",
        "//control/manual:manual_types",
        "//control/trans_in:trans_in_types",
        "//system:labels",
    ],
)

makani_cc_test(
    name = "crosswind_inner_test",
    srcs = ["crosswind_inner_test.cc"],
    deps = [
        ":crosswind_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "crosswind_output_test",
    srcs = ["crosswind_output_test.cc"],
    deps = [
        ":crosswind_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "crosswind_path_test",
    srcs = ["crosswind_path_test.cc"],
    deps = [
        ":crosswind_frame",
        ":crosswind_nostatic",
        "//common/c_math",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "crosswind_power_test",
    srcs = ["crosswind_power_test.cc"],
    deps = [":crosswind_nostatic"],
)

makani_cc_test(
    name = "crosswind_util_test",
    srcs = ["crosswind_util_test.cc"],
    deps = [
        ":crosswind_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "crosswind_playbook_test",
    srcs = ["crosswind_playbook_test.cc"],
    deps = [
        ":crosswind_playbook",
        "//control:system_params",
        "//control:system_types",
        "//lib/util:test_util",
    ],
)

h2py_library(
    name = "h2py_crosswind_types",
    header = "crosswind_types.h",
    header_deps = [":crosswind_types"],
)

makani_c_library(
    name = "crosswind_playbook_types",
    hdrs = ["crosswind_playbook_types.h"],
)

makani_c_library(
    name = "crosswind_playbook",
    srcs = ["crosswind_playbook.c"],
    hdrs = ["crosswind_playbook.h"],
    deps = [
        ":crosswind_playbook_types",
        "//common/c_math",
        "//control:system_params",
        "//control:system_types",
    ],
)
