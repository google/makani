package(default_visibility = ["//visibility:public"])

load("//common:build_rules.bzl", "octave_data")
load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_test",
)
load("//lib/python/pack2:build_rules.bzl", "pack2")

makani_c_library(
    name = "c_math",
    srcs = [
        "kalman.c",
        "optim.c",
    ],
    hdrs = [
        "kalman.h",
        "optim.h",
    ],
    nostatic_files = ["kalman.c"],
    deps = [
        ":common",
        ":linalg",
    ],
)

# This is a GSL-free subset of c_math tailored to TMS570 builds.
makani_c_library(
    name = "common",
    srcs = [
        "coord_trans.c",
        "filter.c",
        "force_moment.c",
        "geometry.c",
        "linalg.h",
        "linalg_common.h",
        "mat2.c",
        "mat3.c",
        "quaternion.c",
        "util.c",
        "vec2.c",
        "vec2f.c",
        "vec3.c",
        "vec3f.c",
        "voting.c",
        "waveform.c",
    ],
    hdrs = [
        "coord_trans.h",
        "filter.h",
        "force_moment.h",
        "geometry.h",
        "mat2.h",
        "mat3.h",
        "quaternion.h",
        "util.h",
        "vec2.h",
        "vec2f.h",
        "vec3.h",
        "vec3f.h",
        "voting.h",
        "waveform.h",
    ],
    nostatic_files = ["voting.c"],
    deps = [
        ":cal_params_c",
        "//common:macros",
    ],
)

makani_c_library(
    name = "linalg",
    srcs = [
        "gsl_linalg_extra.c",
        "linalg.c",
    ],
    hdrs = [
        "gsl_linalg_extra.h",
        "linalg.h",
        "linalg_common.h",
    ],
    linkopts = select({
        "//lib/bazel:q7_mode": ["-Wl,--gc-sections"],
        "//conditions:default": [],
    }),
    deps = select({
        "//lib/bazel:q7_mode": ["//third_party:c_math_gsl"],
        "//conditions:default": ["@usr_lib//:gsl"],
    }),
)

makani_c_library(
    name = "linalg_io",
    srcs = ["util/linalg_io.c"],
    hdrs = ["util/linalg_io.h"],
    deps = [":linalg"],
)

makani_c_library(
    name = "mahony_filter",
    srcs = [
        "mahony_filter.c",
    ],
    hdrs = [
        "mahony_filter.h",
    ],
    nostatic_files = ["mahony_filter.c"],
    deps = [
        ":common",
    ],
)

# Pack2 files.

pack2(
    name = "cal_params",
    src = "cal_params.p2",
)

# Test data.

octave_data(
    m_file = "test_data/constrained_least_squares_comparison.m",
    output = "test_data/constrained_least_squares_comparison.dat",
)

octave_data(
    m_file = "test_data/mat_divide_comparison.m",
    output = "test_data/mat_divide_comparison.dat",
)

octave_data(
    m_file = "test_data/sqrt_kalman_comparison.m",
    output = "test_data/sqrt_kalman_comparison.dat",
)

# Unit tests.

makani_cc_test(
    name = "coord_trans_test",
    srcs = ["coord_trans_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "filter_test",
    srcs = ["filter_test.cc"],
    copts = ["-Wno-float-equal"],
    deps = [
        ":common_nostatic",
        "//lib/util:random_matrix",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "force_moment_test",
    srcs = ["force_moment_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "geometry_test",
    srcs = ["geometry_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "kalman_test",
    srcs = ["kalman_test.cc"],
    data = ["test_data/sqrt_kalman_comparison.dat"],
    deps = [
        ":c_math_nostatic",
        ":linalg_io",
        "//lib/util:random_matrix",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "linalg_test",
    srcs = ["linalg_test.cc"],
    copts = ["-Wno-float-equal"],
    data = ["test_data/mat_divide_comparison.dat"],
    deps = [
        ":common_nostatic",
        ":linalg_io",
        "//lib/util:random_matrix",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "mat2_test",
    srcs = ["mat2_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "mat3_test",
    srcs = ["mat3_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "optim_test",
    srcs = ["optim_test.cc"],
    data = ["test_data/constrained_least_squares_comparison.dat"],
    deps = [
        ":common_nostatic",
        ":linalg_io",
        "//lib/util:random_matrix",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "quaternion_test",
    srcs = ["quaternion_test.cc"],
    copts = ["-Wno-float-equal"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "util_test",
    srcs = ["util_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "vec2_test",
    srcs = ["vec2_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "vec2f_test",
    srcs = ["vec2f_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "vec3f_test",
    srcs = ["vec3f_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "vec3_test",
    srcs = ["vec3_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "voting_test",
    srcs = ["voting_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "waveform_test",
    srcs = ["waveform_test.cc"],
    deps = [
        ":common_nostatic",
        "//lib/util:test_util",
    ],
)

# Python/h2py rules.

h2py_library(
    name = "h2py_filter",
    function_deps = [
        ":common",
    ],
    header = "filter.h",
)

h2py_library(
    name = "h2py_util",
    function_deps = [
        ":common",
    ],
    header = "util.h",
)

h2py_library(
    name = "h2py_geometry",
    function_deps = [
        ":common",
    ],
    header = "geometry.h",
)

h2py_library(
    name = "h2py_coord_trans",
    function_deps = [
        ":common",
    ],
    header = "coord_trans.h",
)
