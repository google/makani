package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:c_rules.bzl", "makani_c_library", "makani_cc_test")
load("//lib/bazel:py_rules.bzl", "py_binary")

# C libraries.

makani_c_library(
    name = "fast_math",
    srcs = [
        "fast_math.c",
        "filter.c",
        "trig_lookup_tables.h",
    ],
    hdrs = [
        "fast_math.h",
        "filter.h",
    ],
    deps = ["//common:macros"],
)

# C/C++ tests.

makani_cc_test(
    name = "fast_math_test",
    srcs = ["fast_math_test.cc"],
    copts = ["-Wno-inline"],
    deps = [
        ":fast_math",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "filter_test",
    srcs = ["filter_test.cc"],
    copts = ["-Wno-inline"],
    deps = [
        ":fast_math",
        "//common:macros",
    ],
)

# Code generation rules.

py_binary(
    name = "generate_trig_lookup_tables",
    srcs = ["trig_lookup_tables.py"],
    main = "trig_lookup_tables.py",
    deps = [
        requirement("numpy"),
        requirement("python-gflags"),
    ],
)

genrule(
    name = "trig_lookup_tables",
    outs = ["trig_lookup_tables.h"],
    cmd = "$(location :generate_trig_lookup_tables) --output_file=$@",
    tools = [":generate_trig_lookup_tables"],
)
