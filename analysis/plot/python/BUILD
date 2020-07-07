package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_library")

py_library(
    name = "pyplot",
    srcs = ["mplot.py"] + glob([
        "plot_groups/*.py",
    ]),
    deps = [
        "//avionics/common:h2py_plc_messages",
        "//control:h2py_control_types",
        "//control:h2py_system_params",
        "//control:h2py_system_types",
        "//lib/python:c_helpers",
        "//lib/python/h5_utils:numpy_utils",
    ],
)
