load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "switch_client",
    srcs = ["switch_client.py"],
    tags = ["tms570_util"],
    deps = [
        "//avionics/common:aio",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/network:network_config",
        "//lib/python:c_helpers",
        "//system:h2py_labels",
    ],
)
