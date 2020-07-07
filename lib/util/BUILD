package(default_visibility = ["//visibility:public"])

load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_library",
    "makani_cc_test",
)

makani_c_library(
    name = "base64",
    srcs = ["base64.c"],
    hdrs = ["base64.h"],
    linkopts = ["-lpcre"],
    deps = ["@usr_include//:glib"],
)

makani_cc_library(
    name = "operator_confirmation",
    srcs = ["operator_confirmation.cc"],
    hdrs = ["operator_confirmation.h"],
)

makani_c_library(
    name = "random_matrix",
    srcs = ["random_matrix.c"],
    hdrs = ["random_matrix.h"],
    linkopts = ["-lgsl"],
    deps = ["//common/c_math"],
)

makani_cc_library(
    name = "test_util",
    srcs = ["test_util.cc"],
    hdrs = ["test_util.h"],
    deps = [
        "//common:macros",
        "//common:runfiles_dir",
        "//common/c_math",
        "@usr_lib//:gflags",
    ],
)

makani_cc_test(
    name = "base64_test",
    srcs = ["base64_test.cc"],
    deps = [
        ":base64",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "circular_buffer_test",
    srcs = glob(["circular_buffer*"]),
)

makani_cc_library(
    name = "string_util",
    srcs = glob(["string_util*"]),
)

makani_cc_test(
    name = "random_matrix_test",
    srcs = ["random_matrix_test.cc"],
    deps = [
        ":random_matrix",
        "//lib/util:test_util",
    ],
)
