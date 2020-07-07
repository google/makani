package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_cc_binary",
    "makani_cc_library",
)
load("//lib/bazel:py_rules.bzl", "py_binary")

# C/C++ targets.

makani_cc_library(
    name = "h5log_reader",
    srcs = [
        "h5log_reader.cc",
        "type_info.cc",
    ],
    hdrs = [
        "h5log_reader.h",
        "type_info.h",
    ],
    deps = [
        "//avionics/common:pack_aio_header",
        "//avionics/common:pack_avionics_messages",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
        "//control:pack_control_telemetry",
        "//control:pack_ground_telemetry",
        "//lib/pcap_to_hdf5:pack_capture_info",
        "//sim:pack_sim_messages",
        "//sim:pack_sim_telemetry",
        "@usr_lib//:glog",
        "@usr_lib//:hdf5",
    ],
)

makani_cc_binary(
    name = "hdf5_to_pcap",
    srcs = ["hdf5_to_pcap.cc"],
    deps = [
        ":h5log_reader",
        "//avionics/common:aio_header",
        "//avionics/common:aio_version",
        "//avionics/common:network_config",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
        "@usr_lib//:pcap",
    ],
)

# Code generation.

py_binary(
    name = "generate_type_info",
    srcs = ["generate_type_info.py"],
    deps = [
        "//avionics/common:aio",
        "//avionics/common:h2py_build_info_types",
        "//avionics/network:aio_message",
        "//control:h2py_control_types",
        "//control:h2py_system_types",
        "//lib/pcap_to_hdf5:h2py_pack_capture_info",
        "//lib/python:ctype_util",
        "//lib/python/autogen:autogen_util",
        "//sim:h2py_sim_types",
    ],
)

genrule(
    name = "genrule_type_info",
    outs = [
        "type_info.cc",
        "type_info.h",
    ],
    cmd = " ".join([
        "$(location :generate_type_info)",
        "--autogen_root=$(GENDIR)",
        "--source_file=$(location type_info.cc)",
        "--header_file=$(location type_info.h)",
    ]),
    tools = [":generate_type_info"],
)
