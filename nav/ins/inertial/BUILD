package(default_visibility = ["//nav/ins:__subpackages__"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_test",
)

# C libraries.

makani_c_library(
    name = "inertial",
    srcs = [
        "inertial.c",
    ],
    hdrs = [
        "inertial.h",
    ],
    deps = [
        ":params",
        ":types",
        "//nav/ins/messages",
        "//nav/ins/messages:labels",
        "//nav/ins/messages:seq_num",
        "//nav/ins/util:ring",
    ],
)

makani_c_library(
    name = "params",
    hdrs = ["params.h"],
)

makani_c_library(
    name = "types",
    srcs = ["types.h"],
    deps = [
        "//common:ring",
        "//nav/ins/messages",
    ],
)

# C++ unit tests.

makani_cc_test(
    name = "inertial_test",
    srcs = ["inertial_test.cc"],
    deps = [
        ":inertial",
        "//nav/ins/messages",
        "//nav/ins/messages:labels",
    ],
)
