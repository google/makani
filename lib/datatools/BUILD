package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_cc_binary",
    "makani_cc_library",
    "makani_cc_test",
)

makani_cc_library(
    name = "load_pcap_hdf5_dataset",
    srcs = ["load_pcap_hdf5_dataset.cc"],
    hdrs = ["load_pcap_hdf5_dataset.h"],
    deps = [
        "@usr_lib//:glog",
        "@usr_lib//:hdf5",
    ],
)

makani_cc_test(
    name = "load_pcap_hdf5_dataset_test",
    srcs = ["load_pcap_hdf5_dataset_test.cc"],
    data = ["//lib/pcap_to_hdf5:empty_log.h5"],
    deps = [
        ":load_pcap_hdf5_dataset",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:pack_control_telemetry",
        "//control:system_params",
        "//lib/util:test_util",
        "//sim:sim_params",
    ],
)

makani_cc_binary(
    name = "validate_pcap",
    srcs = ["validate_pcap.cc"],
    archs = ["x86"],
    deps = [
        "//avionics/common:aio_header",
        "//avionics/common:aio_version",
        "//avionics/common:network_config",
        "//avionics/common:pack_aio_header",
        "//avionics/network:aio_node",
        "//avionics/network:message_stats",
        "//avionics/network:message_type",
        "//common:macros",
        "//control:control_params",
        "//control:control_types",
        "//control:pack_control_telemetry",
        "//control:system_params",
        "//control:system_types",
        "//gs/monitor:monitor_types",
        "//lib/pcap_reader",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
        "@usr_lib//:pcap",
    ],
)
