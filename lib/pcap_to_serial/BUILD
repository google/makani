package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_cc_binary",
)

makani_cc_binary(
    name = "pcap_to_serial",
    srcs = [
        "pcap_to_serial.cc",
    ],
    deps = [
        "//avionics/common:aio_header",
        "//avionics/network:message_info",
        "//lib/pcap_reader",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)
