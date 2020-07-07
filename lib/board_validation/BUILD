package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_library")

py_library(
    name = "base_validator",
    srcs = ["base_validator.py"],
    deps = [
        "//avionics/network:h2py_message_type",
        "//gs/monitor2/apps/receiver:receivers",
        "//lib/python:c_helpers",
    ],
)
