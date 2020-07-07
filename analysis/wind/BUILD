package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "generate_turbsim_database",
    srcs = ["turbsim/generate_turbsim_database.py"],
    data = [
        "turbsim/TurbSim.inp.template",
        "turbsim/convert_turbsim.m",
    ],
    deps = [
        "//lib/python:os_util",
    ],
)
