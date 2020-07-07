package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load(
    "//lib/bazel:autogen_rules.bzl",
    "message_ruleset",
)
load(
    "//lib/bazel:c_rules.bzl",
    "makani_cc_binary",
    "makani_cc_library",
)
load("//lib/bazel:py_rules.bzl", "py_binary")

# This rule comes before pcap_to_hdf5 to ensure a helpful error message if
# hdf5_format.{cc,h} are already in the source tree as Make artifacts.
# (pcap_to_hdf5's glob will trigger an error with a less useful message in that
# case.)
genrule(
    name = "hdf5_format_genrule",
    outs = [
        "format.h5",
        "hdf5_format.cc",
        "hdf5_format.h",
    ],
    cmd = "$(location :generate_hdf5_format) --autogen_root=$(GENDIR)",
    tools = [":generate_hdf5_format"],
)

makani_cc_library(
    name = "message_log",
    srcs = [
        "hdf5_format.cc",
        "hdf5_format.h",
        "message_log.cc",
    ],
    hdrs = [
        "message_log.h",
    ],
    deps = [
        ":capture_info",
        "//avionics/common:aio_version",
        "//avionics/common:build_info",
        "//avionics/common:network_config",
        "//avionics/common:pack_aio_header",
        "//avionics/network:aio_node",
        "//avionics/network:aio_node_to_ip_address",
        "//avionics/network:message_type",
        "//control:control_params",
        "//control:system_params",
        "//lib/pcap_reader",
        "//sim:sim_params",
        "@usr_lib//:hdf5",
    ],
)

makani_cc_binary(
    name = "pcap_to_hdf5",
    srcs = ["pcap_to_hdf5.cc"],
    data = ["format.h5"],
    deps = [
        ":message_log",
        "//avionics/common:aio_header",
        "//avionics/common:aio_version",
        "//avionics/common:network_config",
        "//avionics/common:pack_aio_header",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//control:control_params",
        "//control:control_types",
        "//control:pack_control_telemetry",
        "//control:pack_ground_telemetry",
        "//control:system_params",
        "//control:system_types",
        "//lib/json_load:load_params",
        "//lib/pcap_reader",
        "//sim:pack_sim_telemetry",
        "//sim:sim_params",
        "//sim:sim_types",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
        "@usr_lib//:hdf5",
        "@usr_lib//:pcap",
    ],
)

message_ruleset(
    name = "capture_info",
    generate_cvt_lib = False,
    generate_h2py_lib = True,
    header = "capture_info.h",
    deps = ["//avionics/common:aio_header"],
)

py_binary(
    name = "generate_hdf5_format",
    srcs = ["generate_hdf5_format.py"],
    deps = [
        ":h2py_pack_capture_info",
        "//avionics/common:aio",
        "//avionics/common:h2py_build_info_types",
        "//avionics/common:h2py_pack_aio_header",
        "//avionics/network:aio_message",
        "//control:h2py_control_types",
        "//lib/python/autogen:autogen_util",
        "//sim:h2py_sim_types",
        requirement("h5py"),
    ],
)

# Produces an empty HDF5 log file to provide log structure to tests.
genrule(
    name = "empty_log_genrule",
    outs = ["empty_log.h5"],
    cmd = "$(location :pcap_to_hdf5) --write_empty_log --output_file=$@",
    tools = [":pcap_to_hdf5"],
)
