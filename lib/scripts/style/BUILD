package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "cpplint",
    srcs = ["cpplint.py"],
)

py_binary(
    name = "run_lint",
    srcs = [
        "run_lint.py",
    ],
    data = [
        ":cpplint",
    ],
    deps = [
        "//lib/bazel:bazel_util",
        "//lib/python:os_util",
        requirement("python-gflags"),
    ],
)

py_binary(
    name = "autoformat",
    srcs = ["autoformat.py"],
    deps = [
        "//lib/bazel:bazel_util",
        "//lib/python:os_util",
        requirement("python-gflags"),
    ],
)
