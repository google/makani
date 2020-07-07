package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

py_library(
    name = "json_obj",
    srcs = ["json_obj.py"],
    deps = [
        requirement("jsonpickle"),
    ],
)

py_test(
    name = "json_obj_test",
    srcs = ["json_obj_test.py"],
    deps = [":json_obj"],
)
