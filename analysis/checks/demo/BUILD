package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_library")

py_library(
    name = "samples",
    srcs = ["sample_checks.py"],
    deps = [
        "//analysis/checks:common",
        "//avionics/common:h2py_motor_thermal_types",
    ],
)
