"""This module contains rules for generating Python libraries."""

load("@python_pip//:requirements.bzl", "requirement")
load(
    "@rules_python//python:defs.bzl",
    bazel_py_binary = "py_binary",
    bazel_py_library = "py_library",
    bazel_py_test = "py_test",
)

global_deps = [
    requirement("six"),
]

def py_library(
        name,
        srcs = [],
        deps = [],
        data = [],
        srcs_version = "PY2ONLY",
        **kwargs):
    bazel_py_library(
        name = name,
        srcs = srcs,
        deps = deps + global_deps +
               ["//:pyinit"],
        data = data,
        srcs_version = srcs_version,
        **kwargs
    )

def py_binary(
        name,
        srcs,
        deps = [],
        data = [],
        python_version = "PY2",
        **kwargs):
    bazel_py_binary(
        name = name,
        srcs = srcs,
        deps = deps + global_deps + ["//:pyinit"],
        data = data,
        python_version = python_version,
        **kwargs
    )

def py_test(name, srcs, deps = [], data = [], python_version = "PY2", **kwargs):
    bazel_py_test(
        name = name,
        srcs = srcs,
        deps = deps + global_deps + ["//:pyinit"],
        data = data,
        python_version = python_version,
        **kwargs
    )
