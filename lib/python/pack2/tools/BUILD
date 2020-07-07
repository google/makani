load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "p2fmt",
    srcs = ["p2fmt.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//lib/python/pack2",
        requirement("python-gflags"),
    ],
)

py_binary(
    name = "p2generate",
    srcs = ["p2generate.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//lib/python/pack2",
        requirement("python-gflags"),
    ],
)
