package(default_visibility = ["//visibility:public"])

load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_test",
)

h2py_library(
    name = "h2py_hover_angles",
    function_deps = [":hover_angles"],
    header = "hover_angles.h",
)

makani_c_library(
    name = "hover_types",
    srcs = ["hover_types.c"],
    hdrs = ["hover_types.h"],
    deps = [
        "//avionics/common:plc_messages",
        "//common/c_math",
        "//control:actuator_types",
        "//control/experiments:hover_experiment_types",
    ],
)

makani_c_library(
    name = "hover_frame",
    srcs = ["hover_frame.c"],
    hdrs = ["hover_frame.h"],
    deps = [
        "//common/c_math",
    ],
)

makani_cc_test(
    name = "hover_frame_test",
    srcs = ["hover_frame_test.cc"],
    deps = [
        ":hover_frame",
        "//lib/util:test_util",
    ],
)

makani_c_library(
    name = "hover",
    srcs = glob(
        [
            "*.c",
            "*.h",
        ],
        exclude = [
            "hover_frame.c",
            "hover_frame.h",
            "hover_types.c",
            "hover_types.h",
        ],
    ),
    hdrs = [
        "hover.h",
    ],
    nostatic_files = [
        "hover.c",
    ],
    deps = [
        ":hover_frame",
        ":hover_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
        "//control/crosswind:crosswind_playbook",
        "//control/estimator:estimator_types",
        "//control/experiments:hover_experiment_types",
        "//control/trans_in:trans_in_types",
    ],
)

makani_c_library(
    name = "hover_angles",
    srcs = ["hover_angles.c"],
    hdrs = ["hover_angles.h"],
    nostatic_files = ["hover_angles.c"],
    deps = [
        ":hover_experiments",
        ":hover_frame",
        ":hover_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
    ],
)

makani_c_library(
    name = "hover_inject",
    srcs = ["hover_inject.c"],
    hdrs = ["hover_inject.h"],
    nostatic_files = ["hover_inject.c"],
    deps = [
        ":hover_frame",
        ":hover_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_util",
    ],
)

makani_c_library(
    name = "hover_output",
    srcs = ["hover_output.c"],
    hdrs = ["hover_output.h"],
    nostatic_files = ["hover_output.c"],
    deps = [
        ":hover_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
    ],
)

makani_c_library(
    name = "hover_path",
    srcs = ["hover_path.c"],
    hdrs = ["hover_path.h"],
    nostatic_files = ["hover_path.c"],
    deps = [
        ":hover_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
        "//control/crosswind:crosswind_playbook",
        "//control/estimator:estimator_types",
    ],
)

makani_c_library(
    name = "hover_position",
    srcs = ["hover_position.c"],
    hdrs = ["hover_position.h"],
    nostatic_files = ["hover_position.c"],
    deps = [
        "hover_angles",
        ":hover_frame",
        ":hover_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
    ],
)

makani_c_library(
    name = "hover_tension",
    srcs = ["hover_tension.c"],
    hdrs = ["hover_tension.h"],
    nostatic_files = ["hover_tension.c"],
    deps = [
        ":hover_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
        "//control/estimator:estimator_types",
    ],
)

makani_c_library(
    name = "hover_winch",
    srcs = ["hover_winch.c"],
    hdrs = ["hover_winch.h"],
    nostatic_files = ["hover_winch.c"],
    deps = [
        ":hover_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
        "//control/estimator:estimator_types",
    ],
)

makani_c_library(
    name = "hover_altitude",
    srcs = ["hover_altitude.c"],
    hdrs = ["hover_altitude.h"],
    nostatic_files = ["hover_altitude.c"],
    deps = [
        ":hover_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
        "//control/estimator:estimator_types",
    ],
)

makani_c_library(
    name = "hover_experiments",
    srcs = ["hover_experiments.c"],
    hdrs = ["hover_experiments.h"],
    nostatic_files = ["hover_experiments.c"],
    deps = [
        ":hover_types",
        "//control:control_params",
        "//control:control_types",
        "//control:system_params",
        "//control:system_types",
        "//control/experiments:hover_experiment_types",
    ],
)

makani_cc_test(
    name = "hover_experiments_test",
    srcs = ["hover_experiments_test.cc"],
    deps = [
        # This ordering is intentional due to some build weirdness.
        ":hover_experiments_nostatic",
        ":hover",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "hover_altitude_test",
    srcs = ["hover_altitude_test.cc"],
    deps = ["hover_altitude_nostatic"],
)

makani_cc_test(
    name = "hover_angles_test",
    srcs = ["hover_angles_test.cc"],
    deps = [
        # This ordering is intentional due to some build weirdness.
        ":hover_frame",
        ":hover_angles_nostatic",
        ":hover",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "hover_inject_test",
    srcs = ["hover_inject_test.cc"],
    deps = [
        # This ordering is intentional due to some build weirdness.
        ":hover_inject_nostatic",
        ":hover",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "hover_output_test",
    srcs = ["hover_output_test.cc"],
    deps = [
        ":hover_output_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "hover_path_test",
    srcs = ["hover_path_test.cc"],
    deps = [
        ":hover_frame",
        ":hover_path_nostatic",
        "//control/crosswind:crosswind_playbook",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "hover_position_test",
    srcs = ["hover_position_test.cc"],
    deps = [
        # This ordering is intentional due to some build weirdness.
        ":hover_position_nostatic",
        ":hover_angles_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "hover_tension_test",
    srcs = ["hover_tension_test.cc"],
    deps = [
        ":hover_tension_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "hover_winch_test",
    srcs = ["hover_winch_test.cc"],
    deps = [":hover_winch_nostatic"],
)
