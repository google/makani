package(default_visibility = ["//visibility:public"])

load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load(
    "//lib/bazel:c_rules.bzl",
    "makani_cc_library",
)
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library")

py_binary(
    name = "compute_error_cdfs",
    srcs = ["estimator_helper.py"],
    main = "estimator_helper.py",
    deps = [
        ":h2py_estimator_replay",
        "//control:h2py_control_types",
    ],
)

py_library(
    name = "estimator_helper",
    srcs = ["estimator_helper.py"],
    deps = [
        ":h2py_estimator_replay",
        "//control:h2py_control_types",
    ],
)

makani_cc_library(
    name = "estimator_replay",
    srcs = ["estimator_replay.cc"],
    hdrs = ["estimator_replay.h"],
    deps = [
        "//avionics/common:aio_version",
        "//control:control_system",
        "//control:control_telemetry",
        "//control:control_types",
        "//control:system_types",
        "//lib/hdf5_to_pcap:h5log_reader",
    ],
)

h2py_library(
    name = "h2py_estimator_replay",
    header = "estimator_replay.h",
    header_deps = [
        ":estimator_replay",
    ],
)
