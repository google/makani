package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_test",
)

makani_c_library(
    name = "manual_types",
    hdrs = ["manual_types.h"],
    deps = [
        "//common/c_math",
        "//control:actuator_types",
    ],
)

makani_c_library(
    name = "manual",
    srcs = [
        "manual.c",
        "manual_output.c",
        "manual_output.h",
    ],
    hdrs = ["manual.h"],
    deps = [
        ":manual_types",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_types",
        "//control:control_util",
        "//control:system_types",
        "//control/estimator:estimator_types",
    ],
)

makani_cc_test(
    name = "manual_test",
    srcs = ["manual_test.cc"],
    deps = [
        ":manual",
        "//lib/util:test_util",
    ],
)
