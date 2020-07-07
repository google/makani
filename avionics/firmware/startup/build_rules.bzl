"""This module contains rules for the startup c library."""

load("//lib/bazel:c_rules.bzl", "makani_c_library")

def _get_linkopts(ld_files):
    return (["-Tavionics/firmware/startup/" + f for f in ld_files])

def startup_c_library(name, deps = [], ld_files = [], linkopts = [], **kwargs):
    makani_c_library(
        name = name,
        deps = deps + ld_files,
        linkopts = linkopts + _get_linkopts(ld_files),
        **kwargs
    )
