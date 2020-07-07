package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "joystick",
    srcs = ["joystick.py"],
    deps = [
        "//avionics/common:aio",
        "//avionics/common:h2py_pack_avionics_messages",
        "//lib/python:c_helpers",
        "//sim:h2py_pack_sim_messages",
        "//system:h2py_labels",
        requirement("python-gflags"),
        requirement("wxPython"),
    ],
)
