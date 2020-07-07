package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_test",
)

makani_c_library(
    name = "trans_in_types",
    srcs = ["trans_in_types.c"],
    hdrs = ["trans_in_types.h"],
    deps = [
        "//control:actuator_types",
        "//control:system_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "trans_in",
    srcs = [
        "trans_in.c",
        "trans_in_attitude.c",
        "trans_in_lateral.c",
        "trans_in_longitudinal.c",
        "trans_in_mode.c",
    ],
    hdrs = [
        "trans_in.h",
        "trans_in_attitude.h",
        "trans_in_lateral.h",
        "trans_in_longitudinal.h",
        "trans_in_mode.h",
    ],
    nostatic_files = [
        "trans_in.c",
        "trans_in_longitudinal.c",
    ],
    deps = [
        ":trans_in_types",
        "//common/c_math",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_types",
        "//control:control_util",
        "//control:system_types",
        "//control/estimator:estimator_types",
        "//system:labels",
    ],
)

makani_cc_test(
    name = "trans_in_test",
    srcs = ["trans_in_test.cc"],
    deps = [
        "trans_in_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "trans_in_longitudinal_test",
    srcs = ["trans_in_longitudinal_test.cc"],
    deps = [
        "trans_in_nostatic",
        "//lib/util:test_util",
    ],
)
