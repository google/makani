"""This module contains build rules for batch sims."""

load("@subpar//:subpar.bzl", "parfile")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library")

def batch_sim_ruleset(
        name,
        client_main,
        worker_main,
        extra_srcs = [],
        deps = [],
        client_deps = [],
        data = [],
        **kwargs):
    """Generates two rules for batch sims.

    The two rules generated are:
    - A py_binary with the specified name, with the controller,
        pcap_to_hdf5, and simulator binaries included as data dependencies.
    - A py_library named `name + "_testlib"` that does not include the
        binaries, so they don't get built unnecessarily for unit tests.

    Args:
        name: Name of the rule.
        client_main: `main` argument of the client py_binary rule.
        worker_main: `main` argument of the worker py_binary rule.
        extra_srcs: Additional sources for the py_library.
        deps: List of depencencies for the libraries generated.
        client_deps: List of additional dependencies only for the client
            library.
        data: List of data items for the libraries.
        **kwargs: Additional arguments passed to the py rules.
    """
    srcs = [client_main, worker_main] + extra_srcs
    deps += ["//lib/python/batch_sim:batch_sim"]

    py_library(
        name = name + "_testlib",
        srcs = srcs,
        deps = deps + client_deps,
        data = data,
        **kwargs
    )

    py_binary(
        name = name + "_worker",
        srcs = srcs,
        main = worker_main,
        data = data + [
            "//control:sim_controller",
            "//control:sim_ground_estimator",
            "//lib/pcap_to_hdf5:pcap_to_hdf5",
            "//lib/scripts:sim_tcpdump",
            "//sim:sim",
        ],
        deps = deps,
        **kwargs
    )

    # TODO: Convert to par_binary once custom python hacks are removed.
    parfile(
        name = name + "_worker_package.par",
        src = name + "_worker",
        main = worker_main,
        default_python_version = "PY2",
        zip_safe = False,
    )

    py_binary(
        name = name,
        srcs = srcs,
        main = client_main,
        deps = deps + client_deps,
        data = data + [name + "_worker_package.par"],
        **kwargs
    )
