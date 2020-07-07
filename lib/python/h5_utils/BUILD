package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

py_library(
    name = "h5_utils",
    srcs = [
        "h5_compare.py",
        "h5_io.py",
    ],
    deps = [
        ":numpy_utils",
        requirement("h5py"),
    ],
)

py_library(
    name = "numpy_utils",
    srcs = ["numpy_utils.py"],
    deps = [
        requirement("numpy"),
    ],
)

py_test(
    name = "h5_compare_test",
    srcs = ["h5_compare_test.py"],
    deps = [":h5_utils"],
)

py_test(
    name = "h5_io_test",
    srcs = ["h5_io_test.py"],
    deps = [
        ":h5_utils",
    ],
)

py_test(
    name = "numpy_utils_test",
    srcs = ["numpy_utils_test.py"],
    data = ["//lib/pcap_to_hdf5:empty_log.h5"],
    deps = [
        ":numpy_utils",
        "//lib/python:test_util",
    ],
)
