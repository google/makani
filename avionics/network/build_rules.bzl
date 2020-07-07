"""This module contains rules for the network genrules."""

load("//lib/bazel:c_rules.bzl", "makani_c_library")

# This is a genrule for files that use network.yaml as a source.
def makani_network_genrule(**kwargs):
    kwargs["cmd"] = " ".join([
        "$(location %s) --autogen_root=$(GENDIR)" % kwargs["tools"][0],
        "--output_dir=$(GENDIR)/avionics/network",
        "--network_file=$(location network.yaml)",
    ])
    kwargs["srcs"] = ["network.yaml"]
    native.genrule(**kwargs)

def makani_cvt_genrule(
        name,
        all_nodes = False,
        all_q7s = False,
        all_tms570s = False,
        aio_labels = "",
        aio_nodes = "",
        **kwargs):
    cvt_deps = [
        "//avionics/common:cvt",
        "//avionics/common:cvt_entries",
        "//avionics/common:pack_avionics_messages",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
    ]

    makani_c_library(
        name = name,
        srcs = [
            "cvt_entries_" + name + ".c",
        ],
        deps = select({
            "//lib/bazel:tms570_mode": cvt_deps,
            "//conditions:default": cvt_deps + [
                "//control:pack_control_telemetry",
                "//control:pack_ground_telemetry",
                "//sim:pack_sim_messages",
                "//sim:pack_sim_telemetry",
            ],
        }),
    )

    native.genrule(
        name = "cvt_entries_" + name + "_genrule",
        outs = ["cvt_entries_" + name + ".c"],
        tools = ["//avionics/network:generate_cvt"],
        srcs = ["network.yaml"],
        cmd = " ".join([
            "$(location //avionics/network:generate_cvt)",
            "--all_nodes=" + str(all_nodes),
            "--all_q7s=" + str(all_q7s),
            "--all_tms570s=" + str(all_tms570s),
            "--aio_labels=" + aio_labels,
            "--aio_nodes=" + aio_nodes,
            "--autogen_root=$(GENDIR)",
            "--output_dir=$(GENDIR)/avionics/network",
            "--output_source=cvt_entries_" + name + ".c",
            "--network_file=$(location network.yaml)",
        ]),
        **kwargs
    )
