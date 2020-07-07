package(default_visibility = ["//visibility:private"])

load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

py_library(
    name = "site",
    srcs = ["site.py"],
)

py_test(
    name = "site_test",
    srcs = ["site_test.py"],
    deps = [":site"],
)
