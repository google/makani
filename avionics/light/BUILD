package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "light_client",
    srcs = ["light_client.py"],
    data = ["//avionics/firmware/drivers:faa_light.c"],
    tags = ["tms570_util"],
    deps = [
        "//avionics/common:aio",
        "//avionics/common:cmd_client",
        "//avionics/common:h2py_pack_avionics_messages",
    ],
)
