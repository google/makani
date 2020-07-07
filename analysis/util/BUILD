package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:c_rules.bzl", "makani_cc_binary")
load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "batch_sim_comparison",
    srcs = [
        "batchsim_analysis_util.py",
        "batchsim_histograms.py",
    ],
    data = [
        "//lib/python/batch_sim:batch_sim_comparison_resources",
    ],
    main = "batchsim_histograms.py",
    deps = [
    ],
)

makani_cc_binary(
    name = "dump_sim_telemetry",
    srcs = ["dump_sim_telemetry.cc"],
    deps = [
        "//avionics/common:pack_avionics_messages",
        "//avionics/linux:aio",
        "//avionics/linux:cvt_util",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
        "//control:control_types",
        "//control:system_types",
        "//lib/datatools:load_pcap_hdf5_dataset",
        "//sim:cvt_sim_messages",
        "//sim:pack_sim_telemetry",
        "//sim:sim_telemetry",
        "//sim:sim_types",
        "//system:labels",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

py_binary(
    name = "h5comp",
    srcs = ["h5comp.py"],
    deps = [
        "//lib/python:algorithms",
        "//lib/python/h5_utils",
        requirement("h5py"),
        requirement("python-gflags"),
    ],
)

py_binary(
    name = "plot_regression_output",
    srcs = ["plot_regression_output.py"],
    data = ["regression_dashboard.html"],
    deps = [
        "//control:h2py_common",
        "//control:h2py_control_types",
        "//lib/python:c_helpers",
        requirement("h5py"),
        requirement("matplotlib"),
        requirement("python-gflags"),
        requirement("scipy"),
    ],
)

py_binary(
    name = "validate_flight",
    srcs = ["validate_flight.py"],
    deps = [
        "//config:mconfig",
        "//control:h2py_control_types",
        "//lib/python:c_helpers",
        "//lib/python/h5_utils",
        requirement("h5py"),
        requirement("numpy"),
        requirement("python-gflags"),
    ],
)
