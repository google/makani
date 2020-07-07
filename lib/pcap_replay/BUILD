package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_cc_binary",
)

makani_cc_binary(
    name = "pcap_replay",
    srcs = [
        "pcap_replay.cc",
        "pcap_replay.h",
    ],
    deps = [
        "//avionics/linux:aio_socket",
        "//avionics/linux:clock",
        "//lib/pcap_reader",
        "//lib/util:string_util",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)
