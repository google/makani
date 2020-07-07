package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library")

py_library(
    name = "autogen_util",
    srcs = ["autogen_util.py"],
    srcs_version = "PY2AND3",
)

py_binary(
    name = "clang2py",
    srcs = ["clang2py.py"],
    python_version = "PY3",
    deps = [
        requirement("clang"),
        requirement("ctypeslib2"),
    ],
)

py_binary(
    name = "h2py",
    srcs = ["h2py.py"],
    data = [":clang2py"],
    python_version = "PY3",
    deps = [
        ":autogen_util",
        requirement("python-gflags"),
    ],
)
