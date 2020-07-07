package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_test")
load("//lib/python/batch_sim:build_rules.bzl", "batch_sim_ruleset")

batch_sim_ruleset(
    name = "hover_disturbances_batch_sim",
    client_deps = [
        "//lib/python/batch_sim:parameter_tables",
    ],
    client_main = "hover_disturbances/batch_sim_client.py",
    data = ["//lib/python/batch_sim:overrides_table_style"],
    extra_srcs = ["hover_disturbances/batch_sim_params.py"],
    worker_main = "hover_disturbances/batch_sim_worker.py",
    deps = [
        "//config:mconfig",
        "//lib/python:flag_types",
        "//lib/python/batch_sim:scoring_functions_worker",
    ],
)

py_test(
    name = "hover_disturbances_test",
    srcs = ["hover_disturbances/batch_sim_test.py"],
    data = ["//lib/pcap_to_hdf5:empty_log.h5"],
    main = "hover_disturbances/batch_sim_test.py",
    deps = [
        ":hover_disturbances_batch_sim_testlib",
        "//lib/python/batch_sim:scoring_functions_worker",
        "//lib/python/batch_sim:test_util",
    ],
)

batch_sim_ruleset(
    name = "iec_cases_batch_sim",
    client_main = "iec_cases/batch_sim_client.py",
    worker_main = "iec_cases/batch_sim_worker.py",
    deps = [
        "//config:mconfig",
        requirement("matplotlib"),
    ],
)

# TODO(b/129163698): Figure out why this test is taking longer.
py_test(
    name = "iec_cases_test",
    timeout = "long",
    srcs = ["iec_cases/batch_sim_test.py"],
    data = ["//lib/pcap_to_hdf5:empty_log.h5"],
    main = "iec_cases/batch_sim_test.py",
    deps = [
        ":iec_cases_batch_sim_testlib",
        "//lib/python/batch_sim:test_util",
    ],
)

batch_sim_ruleset(
    name = "power_curve_batch_sim",
    client_main = "power_curve/batch_sim_client.py",
    data = ["power_curve/power_curve_dashboard.html"],
    worker_main = "power_curve/batch_sim_worker.py",
    deps = [
        "//config:mconfig",
        "//lib/python:build_info",
        "//lib/python:flag_types",
        "//lib/python/batch_sim:flight_modes",
        requirement("matplotlib"),
    ],
)

py_test(
    name = "power_curve_test",
    srcs = ["power_curve/batch_sim_test.py"],
    data = ["//lib/pcap_to_hdf5:empty_log.h5"],
    main = "power_curve/batch_sim_test.py",
    deps = [
        ":power_curve_batch_sim_testlib",
        "//lib/python/batch_sim:flight_modes",
        "//lib/python/batch_sim:test_util",
    ],
)
