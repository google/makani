package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library", "py_test")

py_library(
    name = "avl_reader",
    srcs = ["avl_reader.py"],
    data = glob(["*.avl"]),
    deps = [
        requirement("numpy"),
    ],
)

py_binary(
    name = "make_aero_database",
    srcs = ["make_aero_database.py"],
    data = [
        "m600_low_tail_no_winglets.avl",
        "m600_low_tail_no_winglets.mass",
        "//analysis/aero:airfoils",
        "//third_party/avl",
    ],
    deps = ["//lib/python:os_util"],
)

py_test(
    name = "avl_reader_test",
    srcs = ["avl_reader_test.py"],
    deps = [":avl_reader"],
)
