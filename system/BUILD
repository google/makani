package(default_visibility = ["//visibility:public"])

load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load("//lib/bazel:c_rules.bzl", "makani_c_library")

makani_c_library(
    name = "labels",
    srcs = ["labels_util.c"],
    hdrs = [
        "labels.h",
        "labels_util.h",
    ],
    deps = [
        "//avionics/common:gps_receiver_c",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
    ],
)

h2py_library(
    name = "h2py_labels",
    header = "labels.h",
    header_deps = [":labels"],
)

h2py_library(
    name = "h2py_labels_util",
    function_deps = [":labels"],
    header = "labels_util.h",
)
