package(default_visibility = ["//visibility:public"])

load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load("//lib/bazel:c_rules.bzl", "makani_c_library")

makani_c_library(
    name = "experiment_types",
    srcs = ["experiment_types.c"],
    hdrs = ["experiment_types.h"],
    deps = [
        ":crosswind_experiment_types",
        ":hover_experiment_types",
    ],
)

makani_c_library(
    name = "experiment_util",
    srcs = ["experiment_util.c"],
    hdrs = ["experiment_util.h"],
    deps = [
        ":experiment_types",
        "//control:control_params",
    ],
)

makani_c_library(
    name = "hover_experiment_types",
    hdrs = ["hover_experiment_types.h"],
)

makani_c_library(
    name = "crosswind_experiment_types",
    hdrs = ["crosswind_experiment_types.h"],
)

makani_c_library(
    name = "crosswind_experiment",
    srcs = ["crosswind_experiment.c"],
    hdrs = ["crosswind_experiment.h"],
)

h2py_library(
    name = "h2py_experiment_types",
    header = "experiment_types.h",
    header_deps = [":experiment_types"],
)
