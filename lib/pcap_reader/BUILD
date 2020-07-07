load(
    "//lib/bazel:c_rules.bzl",
    "makani_cc_library",
    "makani_cc_test",
)

# C/C++ libraries.

makani_cc_library(
    name = "aio_reader",
    srcs = ["aio_reader.cc"],
    hdrs = ["aio_reader.h"],
    deps = [
        ":udp_reader",
        "//avionics/common:aio_header",
        "//avionics/common:aio_version",
        "//avionics/common:network_config",
        "//avionics/common:pack_aio_header",
        "//avionics/network:aio_node",
        "//avionics/network:message_info",
        "//avionics/network:message_type",
    ],
)

makani_cc_library(
    name = "ip_defragmenter",
    srcs = ["ip_defragmenter.cc"],
    hdrs = ["ip_defragmenter.h"],
    deps = [
        "//common:macros",
        "@usr_lib//:glog",
    ],
)

makani_cc_library(
    name = "ip_reader",
    srcs = ["ip_reader.cc"],
    hdrs = ["ip_reader.h"],
    deps = [
        ":ip_defragmenter",
        "@usr_lib//:pcap",
    ],
)

makani_cc_library(
    name = "pcap_reader",
    srcs = ["pcap_reader.cc"],
    hdrs = ["pcap_reader.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":aio_reader",
        ":ip_reader",
        ":udp_reader",
        "@usr_lib//:glog",
    ],
)

makani_cc_library(
    name = "udp_reader",
    srcs = ["udp_reader.cc"],
    hdrs = ["udp_reader.h"],
    deps = [
        ":ip_reader",
    ],
)

# C/C++ unit tests.

makani_cc_test(
    name = "ip_defragmenter_test",
    srcs = ["ip_defragmenter_test.cc"],
    deps = [":ip_defragmenter"],
)
