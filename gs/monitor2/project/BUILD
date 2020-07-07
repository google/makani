package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_library")

py_library(
    name = "webmonitor_settings",
    srcs = glob(["**/*.py"]),
    deps = [
        "//avionics/network:network_config",
        "//config:mconfig_light",
    ],
)
