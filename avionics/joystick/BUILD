package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "joystick",
    srcs = ["joystick.py"],
    tags = ["tms570_util"],
    deps = [
        "//avionics/common:aio",
        "//system:h2py_labels",
    ],
)
