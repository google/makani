package(default_visibility = ["//visibility:public"])

load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "ground_power_client",
    srcs = ["ground_power_client.py"],
    data = [
        "//avionics/ground_power/q7:flags.h",
        "//avionics/ground_power/q7:inverter_types.h",
    ],
    deps = [
        "//avionics/common:aio",
        "//avionics/common:cmd_client",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/common:h2py_safety_codes",
        "//avionics/ground_power/q7:h2py_flags",
        "//avionics/ground_power/q7:h2py_inverter_types",
        "//avionics/network:h2py_aio_labels",
        "//gs/monitor2/apps/plugins/indicators",
    ],
)
