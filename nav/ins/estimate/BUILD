package(default_visibility = ["//nav/ins:__subpackages__"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_test",
)

# C libraries.

makani_c_library(
    name = "estimate",
    srcs = [
        "estimate.c",
    ],
    hdrs = [
        "estimate.h",
    ],
    deps = [
        ":types",
        "//nav/ins/inertial",
        "//nav/ins/messages",
    ],
)

makani_c_library(
    name = "types",
    srcs = ["types.h"],
)

# C++ unit tests.

makani_cc_test(
    name = "estimate_test",
    srcs = ["estimate_test.cc"],
    deps = [
        ":estimate",
        "//nav/ins/inertial",
        "//nav/ins/messages",
        "//nav/ins/messages:labels",
    ],
)
