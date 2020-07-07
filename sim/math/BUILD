package(default_visibility = ["//visibility:public"])

load("//lib/bazel:c_rules.bzl", "makani_cc_library", "makani_cc_test")

makani_cc_library(
    name = "math",
    srcs = [
        "linalg.cc",
        "ode_solver_gsl.cc",
        "ode_solver_odeint.cc",
        "signal.cc",
        "util.cc",
    ],
    hdrs = [
        "linalg.h",
        "ode_solver.h",
        "ode_solver_gsl.h",
        "ode_solver_odeint.h",
        "signal.h",
        "util.h",
    ],
    deps = [
        "//common:macros",
        "//common/c_math",
        "//sim:sim_types",
        "@usr_lib//:glog",
    ],
)

makani_cc_library(
    name = "interp",
    srcs = [
        "interp.cc",
    ],
    hdrs = [
        "interp.h",
    ],
    deps = [
        "//common/c_math",
    ],
)

makani_cc_test(
    name = "interp_test",
    srcs = ["interp_test.cc"],
    deps = [
        ":interp",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "linalg_test",
    srcs = ["linalg_test.cc"],
    deps = [":math"],
)

makani_cc_test(
    name = "signal_test",
    srcs = ["signal_test.cc"],
    deps = [
        ":math",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "util_test",
    srcs = ["util_test.cc"],
    deps = [":math"],
)
