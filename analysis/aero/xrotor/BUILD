load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "generate_rotor_database",
    srcs = ["generate_rotor_database.py"],
    data = [
        "rf7w6.xrot",
        "//third_party/xrotor",
    ],
    deps = ["//lib/python:flag_types"],
)
