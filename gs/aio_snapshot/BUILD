package(default_visibility = ["//visibility:public"])

load("//lib/bazel:autogen_rules.bzl", "message_ruleset")
load("//lib/bazel:c_rules.bzl", "makani_cc_binary")

# This is currently needed by message_ruleset's implementation.

message_ruleset(
    name = "aio_telemetry",
    generate_cvt_lib = False,
    generate_h2py_lib = False,
    header = "aio_telemetry.h",
    deps = [
        "//avionics/common:avionics_messages",
        "//control:control_telemetry",
        "//control:system_types",
        "//system:labels",
    ],
)

makani_cc_binary(
    name = "aio_snapshot",
    srcs = [
        "aio_snapshot.cc",
        "aio_snapshot.h",
    ],
    deps = [
        ":pack_aio_telemetry",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/linux:aio",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
        "//control:cvt_control_telemetry",
        "//control:system_params",
        "//control:system_types",
        "//control/avionics:avionics_sim",
        "//gs/monitor:monitor_params",
        "//lib/udpio",
        "//sim:cvt_sim_messages",
        "//system:labels",
        "@usr_lib//:glog",
    ],
)
