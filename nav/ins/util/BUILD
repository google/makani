load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
)

# C libraries.

makani_c_library(
    name = "ring",
    hdrs = ["ring.h"],
    visibility = [
        "//nav/ins:__subpackages__",
    ],
    deps = [
        "//common:ring",
    ],
)
