load("//lib/bazel:py_rules.bzl", "py_test")
load("//lib/python/pack2:build_rules.bzl", "pack2")

pack2(
    name = "test_param_parent",
    src = "test_param_parent.p2",
)

pack2(
    name = "test_param",
    src = "test_param.p2",
    deps = [":test_param_parent"],
)

py_test(
    name = "backend_py_test",
    srcs = ["backend_py_test.py"],
    deps = [":test_param_py"],
)

py_test(
    name = "p2fmt_test",
    srcs = ["p2fmt_test.py"],
    data = [
        "test_param.p2",
        "test_param_parent.p2",
        "test_param_post_fmt.p2",
        "test_param_pre_fmt.p2",
    ],
    deps = ["//lib/python/pack2"],
)

py_test(
    name = "parser_test_bitfield",
    srcs = ["parser_test_bitfield.py"],
    deps = ["//lib/python/pack2"],
)

py_test(
    name = "parser_test_enum",
    srcs = ["parser_test_enum.py"],
    deps = ["//lib/python/pack2"],
)

py_test(
    name = "parser_test_include",
    srcs = ["parser_test_include.py"],
    deps = ["//lib/python/pack2"],
)

py_test(
    name = "parser_test_misc",
    srcs = ["parser_test_misc.py"],
    deps = ["//lib/python/pack2"],
)

py_test(
    name = "parser_test_scaled",
    srcs = ["parser_test_scaled.py"],
    deps = ["//lib/python/pack2"],
)

py_test(
    name = "parser_test_specialized",
    srcs = ["parser_test_specialized.py"],
    deps = ["//lib/python/pack2"],
)

py_test(
    name = "parser_test_struct",
    srcs = ["parser_test_struct.py"],
    deps = ["//lib/python/pack2"],
)

py_test(
    name = "source_test",
    srcs = ["source_test.py"],
    data = [
        "test_param.p2",
        "test_param_parent.p2",
    ],
    deps = ["//lib/python/pack2"],
)
