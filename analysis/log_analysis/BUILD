package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_binary", "py_library")

py_library(
    name = "check_util",
    srcs = ["check_util.py"],
    deps = [
        "//analysis/checks:common",
        "//lib/python:import_util",
    ],
)

py_library(
    name = "loop_averager",
    srcs = ["loop_averager.py"],
)

py_binary(
    name = "analyze_logs",
    srcs = ["analyze_logs.py"],
    deps = [
        ":check_util",
        "//analysis/checks:autocheck",
        "//analysis/checks/collection",
        "//analysis/checks/demo:samples",
        "//analysis/crosswind_batch_sims:crosswind_sweep_checks",
        "//lib/python:json_util",
        "//lib/python:struct_tree",
        "//lib/python/batch_sim:gcloud_util",
    ],
)

py_binary(
    name = "time_logs",
    srcs = ["time_logs.py"],
    deps = [
        "//analysis/checks:check_range",
        "//analysis/checks:log_util",
        "//avionics/motor/firmware:h2py_flags",
        "//avionics/network:h2py_aio_labels",
        "//avionics/network:network_config",
        "//lib/python:c_helpers",
        "//lib/python:gsutil",
        "//lib/python:json_util",
        "//lib/python:struct_tree",
        "//lib/python/batch_sim:gcloud_util",
    ],
)
