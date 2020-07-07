package(default_visibility = ["//visibility:public"])

load("//lib/bazel:c_rules.bzl", "makani_cc_binary")

makani_cc_binary(
    name = "flight_gear_interface",
    srcs = glob(["*.cc"]),
    deps = [
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
        "//common/c_math",
        "//control:control_types",
        "//control:cvt_control_telemetry",
        "//control:system_params",
        "//control:system_types",
        "//lib/udpio",
        "//sim:cvt_sim_telemetry",
        "//sim:sim_params",
        "//sim:sim_types",
        "@usr_lib//:gflags",
    ],
)
