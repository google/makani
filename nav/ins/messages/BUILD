package(
    default_visibility = [
        "//nav/aio:__subpackages__",
        "//nav/ins:__subpackages__",
    ],
)

load(
    "//lib/bazel:autogen_rules.bzl",
    "message_ruleset",
)
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_test",
)

makani_c_library(
    name = "labels",
    hdrs = ["labels.h"],
)

makani_c_library(
    name = "message_header",
    hdrs = ["message_header.h"],
    deps = [":message_types"],
)

message_ruleset(
    name = "message_structs",
    generate_cvt_lib = False,
    generate_h2py_lib = True,
    header = "message_structs.h",
    deps = [":message_types"],
)

makani_c_library(
    name = "message_types",
    hdrs = ["message_types.h"],
)

makani_c_library(
    name = "messages",
    hdrs = ["messages.h"],
    deps = [
        ":message_header",
        ":message_structs",
    ],
)

makani_c_library(
    name = "seq_num",
    srcs = ["seq_num.c"],
    hdrs = ["seq_num.h"],
)

# C++ unit tests.

makani_cc_test(
    name = "seq_num_test",
    srcs = ["seq_num_test.cc"],
    deps = [
        ":seq_num",
    ],
)
