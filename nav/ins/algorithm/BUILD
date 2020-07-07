package(default_visibility = ["//nav/aio:__subpackages__"])

load("//lib/bazel:c_rules.bzl", "makani_c_library", "makani_cc_test")

# C libraries.

makani_c_library(
    name = "algorithm",
    srcs = ["algorithm.c"],
    hdrs = ["algorithm.h"],
    deps = [
        ":params",
        ":types",
        "//nav/ins/estimate:types",
        "//nav/ins/inertial",
        "//nav/ins/messages",
        "//nav/ins/output",
    ],
)

makani_c_library(
    name = "params",
    hdrs = ["params.h"],
    deps = [
        "//nav/ins/inertial:params",
    ],
)

makani_c_library(
    name = "types",
    hdrs = ["types.h"],
    deps = [
        "//nav/ins/inertial:types",
        "//nav/ins/output:types",
    ],
)

# C++ unit tests.

makani_cc_test(
    name = "algorithm_test",
    srcs = ["algorithm_test.cc"],
    deps = [
        ":algorithm",
    ],
)
