package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_library")

py_library(
    name = "common",
    srcs = ["common.py"],
)

py_library(
    name = "webmonitor_plugins",
    # A placeholder for future modules.
    srcs = [],
    deps = [
        "//gs/monitor2/apps/plugins/indicators",
        "//gs/monitor2/apps/plugins/layouts",
    ],
)
