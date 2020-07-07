package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/bazel:py_rules.bzl", "py_library", "py_test")
load("//lib/bazel:swig_rules.bzl", "py_wrap_c")

makani_c_library(
    name = "aio_util",
    srcs = ["aio_util.c"],
    hdrs = ["aio_util.h"],
    deps = [
        "//avionics/common:cvt_avionics_messages",
        "//avionics/common:tether_convert",
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//control:system_params",
        "//control/avionics:avionics_sim",
        "//gs/monitor2/high_frequency_filters:common",
        "//sim:pack_sim_messages",
    ],
)

# TODO: In principle, we should have a way to group together
# dependence on the SWIG interface file numpy.i and the cc_library
# numpy_headers. As is, they must be specified separately.
py_wrap_c(
    name = "pywrap_aio_util",
    c_deps = [
        ":aio_util",
        "//third_party/swig_lib:numpy_headers",
    ],
    copts = ["-Wno-pedantic"],
    defines = ["__linux__"],
    module_name = "aio_util",
    py_deps = [
        requirement("numpy"),
    ],
    swig_interfaces = [
        "aio_util.i",
        "//third_party/swig_lib:numpy.i",
    ],
)

py_library(
    name = "aio_helper",
    srcs = ["aio_helper.py"],
    deps = [
        ":pywrap_aio_util",
        "//avionics/common:aio",
        "//avionics/common:h2py_network_config",
        "//avionics/network:h2py_aio_node",
        "//avionics/network:h2py_message_type",
        "//gs/monitor2/high_frequency_filters:h2py_filter_types",
    ],
)

py_test(
    name = "aio_helper_test",
    srcs = ["aio_helper_test.py"],
    tags = ["exclusive"],
    deps = [
        ":aio_helper",
        "//avionics/network:h2py_aio_node",
        "//avionics/network:h2py_message_type",
    ],
)
