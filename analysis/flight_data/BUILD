package(default_visibility = ["//visibility:public"])

load("//lib/bazel:c_rules.bzl", "makani_cc_binary", "makani_cc_library")
load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "generate_wind_db_from_flight_log",
    srcs = ["generate_wind_db_from_flight_log.py"],
    deps = [
        "//config:mconfig",
        "//control:h2py_system_types",
        "//lib/python:c_helpers",
        "//lib/python:h5log_loader",
        "//sim/physics:h2py_wind_frame",
    ],
)

py_binary(
    name = "h5_to_google_earth",
    srcs = ["h5_to_google_earth.py"],
    deps = [
        "//common/c_math:h2py_coord_trans",
        "//control:h2py_control_types",
        "//control:h2py_ground_frame",
        "//lib/python:c_helpers",
    ],
)

makani_cc_library(
    name = "dynamics_sim",
    srcs = ["dynamics_sim.cc"],
    hdrs = ["dynamics_sim.h"],
    deps = [
        "//control:common",
        "//control:control_telemetry",
        "//control:sensor_types",
        "//control:system_params",
        "//control/avionics:avionics_interface",
        "//sim:pack_sim_telemetry",
        "//sim:sim_params",
        "//sim/models:base_system_model",
        "//sim/models:environment",
        "//sim/models:rotor",
        "//sim/models:wing",
        "//sim/physics:aero_frame",
        "//sim/physics:ground_frame",
    ],
)

makani_cc_binary(
    name = "dynamics_replay",
    srcs = ["dynamics_replay.cc"],
    deps = [
        ":dynamics_sim",
        "//analysis/control/replay:estimator_replay",
        "//avionics/linux:clock",
        "//common:runfiles_dir",
        "//lib/hdf5_to_pcap:h5log_reader",
        "//lib/pcap_to_hdf5:message_log",
    ],
)

makani_cc_binary(
    name = "estimator_replay",
    srcs = ["estimator_replay.cc"],
    deps = [
        "//analysis/control/replay:estimator_replay",
        "//avionics/linux:clock",
        "//common:runfiles_dir",
        "//control:control_telemetry",
        "//lib/hdf5_to_pcap:h5log_reader",
        "//lib/pcap_to_hdf5:message_log",
        "//sim:pack_sim_messages",
    ],
)
