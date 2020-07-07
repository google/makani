package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_binary", "py_library", "py_test")
load("//lib/python/batch_sim:build_rules.bzl", "batch_sim_ruleset")

py_library(
    name = "crosswind_batch_sim_params",
    srcs = ["batch_sim_params.py"],
    deps = [
        "//control:h2py_control_params",
        "//lib/python/batch_sim:flight_modes",
        "//lib/python/batch_sim:parameter_sweeps",
        "//lib/python/batch_sim:scoring_functions",
        "//lib/python/h5_utils:numpy_utils",
    ],
)

batch_sim_ruleset(
    name = "crosswind_sweeps_batch_sim",
    client_deps = [
        "//lib/python/batch_sim:parameter_tables",
    ],
    client_main = "crosswind_sweeps/crosswind_sweeps_client.py",
    data = ["//lib/python/batch_sim:overrides_table_style"],
    worker_main = "crosswind_sweeps/crosswind_sweeps_worker.py",
    deps = [
        ":crosswind_batch_sim_params",
        "//config:mconfig",
        "//lib/python:c_helpers",
        "//lib/python:flag_types",
        "//lib/python:turbsim_util",
        "//lib/python/batch_sim:scoring_functions_worker",
    ],
)

py_test(
    name = "crosswind_sweeps_test",
    size = "large",
    srcs = ["crosswind_sweeps/crosswind_sweeps_test.py"],
    data = ["//lib/pcap_to_hdf5:empty_log.h5"],
    deps = [
        ":crosswind_sweeps_batch_sim_testlib",
        "//lib/python/batch_sim:test_util",
    ],
)

py_library(
    name = "crosswind_sweep_checks",
    srcs = ["crosswind_sweep_checks.py"],
    deps = [
        ":crosswind_batch_sim_params",
        "//analysis/checks/collection",
    ],
)

py_binary(
    name = "score_log_file",
    srcs = ["score_log_file.py"],
    deps = [
        ":crosswind_batch_sim_params",
    ],
)
