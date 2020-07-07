package(default_visibility = ["//nav/aio:__subpackages__"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_test",
)

# C libraries.

makani_c_library(
    name = "aio_to_ins",
    srcs = [
        "aio_to_ins.c",
        "flight_computer_sensor.c",
    ],
    hdrs = [
        "aio_to_ins.h",
        "flight_computer_sensor.h",
    ],
    deps = [
        ":types",
        "//avionics/common:aio_header",
        "//avionics/network:message_info",
        "//common:macros",
        "//nav/ins/messages",
        "//nav/ins/messages:message_types",
    ],
)

makani_c_library(
    name = "label_map",
    srcs = ["label_map.c"],
    hdrs = ["label_map.h"],
    deps = [
        ":types",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//nav/ins/messages:labels",
        "//nav/ins/messages:message_types",
    ],
)

makani_c_library(
    name = "types",
    hdrs = ["types.h"],
    deps = [
        "//avionics/network:aio_node",
        "//nav/ins/messages",
        "//nav/ins/messages:message_types",
    ],
)

# C/C++ unit tests.

makani_cc_test(
    name = "aio_to_ins_test",
    srcs = ["aio_to_ins_test.cc"],
    deps = [
        ":aio_to_ins",
        ":label_map",
        "//avionics/common:aio_header",
        "//avionics/common:avionics_messages",
        "//avionics/network:aio_node",
        "//nav/ins/messages",
    ],
)

makani_cc_test(
    name = "flight_computer_sensor_test",
    srcs = ["flight_computer_sensor_test.cc"],
    deps = [
        ":aio_to_ins",
        ":label_map",
        "//avionics/common:aio_header",
        "//avionics/common:avionics_messages",
        "//avionics/network:aio_node",
        "//common:macros",
        "//nav/ins/messages",
    ],
)
