package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_test")

py_binary(
    name = "hw_helper",
    srcs = ["hw_helper.py"],
)

py_binary(
    name = "nm_helper",
    srcs = ["nm_helper.py"],
)

py_binary(
    name = "process_helper",
    srcs = ["process_helper.py"],
)

py_binary(
    name = "ui_helper",
    srcs = ["ui_helper.py"],
    deps = [
        requirement("python2-pythondialog"),
    ],
)

py_test(
    name = "hw_helper_test",
    srcs = ["hw_helper_test.py"],
    deps = [
        ":hw_helper",
        "//lib/python:os_util",
        requirement("mock"),
    ],
)

py_test(
    name = "nm_helper_test",
    srcs = ["nm_helper_test.py"],
    deps = [
        ":nm_helper",
        "//lib/python:os_util",
        requirement("mock"),
    ],
)
