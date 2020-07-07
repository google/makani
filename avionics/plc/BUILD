package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "detwist_reference",
    srcs = ["detwist_reference.py"],
    deps = [
        "//avionics/common:aio",
        "//avionics/common:cmd_client",
        "//avionics/common:h2py_actuator_types",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/common:h2py_pack_plc_messages",
        "//avionics/common:h2py_plc_messages",
        "//system:h2py_labels",
    ],
)
