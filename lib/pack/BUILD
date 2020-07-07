package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "pack",
    srcs = ["pack.py"],
    data = ["//lib/python/autogen:clang2py"],
    deps = [
        "//lib/python/autogen:autogen_util",
        requirement("python-gflags"),
    ],
)
