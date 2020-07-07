load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "gtest_lib",
    srcs = [
        "googlemock/src/gmock-all.cc",
        "googletest/src/gtest-all.cc",
    ],
    hdrs = glob([
        "**/*.h",
        "googletest/src/*.cc",
        "googlemock/src/*.cc",
    ]),
    copts = [
        "-Wno-conversion",
        "-Wno-effc++",
        "-Wno-suggest-attribute=format",
        "-Wno-switch-enum",
        "-Wno-undef",
    ],
    includes = [
        "googlemock",
        "googlemock/include",
        "googletest",
        "googletest/include",
    ],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)
