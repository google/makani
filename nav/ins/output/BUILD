package(default_visibility = ["//nav/ins/algorithm:__subpackages__"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_test",
)

# C libraries.

makani_c_library(
    name = "output",
    srcs = [
        "output.c",
    ],
    hdrs = [
        "output.h",
    ],
    deps = [
        ":types",
        "//nav/ins/estimate",
        "//nav/ins/inertial:types",
        "//nav/ins/messages",
    ],
)

makani_c_library(
    name = "types",
    srcs = ["types.h"],
    deps = [
        "//common:ring",
        "//nav/ins/estimate:types",
        "//nav/ins/messages:message_types",
    ],
)

# C++ unit tests.

makani_cc_test(
    name = "output_test",
    srcs = ["output_test.cc"],
    deps = [
        ":output",
        "//nav/ins/inertial",
    ],
)
