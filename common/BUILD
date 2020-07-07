package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_library",
    "makani_cc_test",
)

makani_cc_library(
    name = "backtrace",
    srcs = ["backtrace.cc"],
    hdrs = ["backtrace.h"],
)

makani_c_library(
    name = "macros",
    hdrs = ["macros.h"],
)

makani_c_library(
    name = "barrier",
    hdrs = ["barrier.h"],
)

makani_c_library(
    name = "ring",
    hdrs = ["ring.h"],
    deps = [
        ":barrier",
    ],
)

makani_cc_library(
    name = "runfiles_dir",
    srcs = ["runfiles_dir.cc"],
    hdrs = ["runfiles_dir.h"],
    deps = [
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

# Unit tests.

makani_cc_test(
    name = "ring_test",
    srcs = ["ring_test.cc"],
    copts = ["-Wno-inline"],
    deps = [
        ":ring",
    ],
)
