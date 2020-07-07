load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "generate_airfoil_database",
    srcs = ["generate_airfoil_database.py"],
    data = [
        "//third_party/xfoil",
    ],
    deps = [
        "//lib/python:flag_types",
        "//lib/python:os_util",
    ],
)
