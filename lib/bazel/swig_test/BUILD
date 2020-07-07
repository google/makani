package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:c_rules.bzl", "makani_c_library", "makani_cc_library")
load("//lib/bazel:py_rules.bzl", "py_test")
load("//lib/bazel:swig_rules.bzl", "py_wrap_c", "py_wrap_cc")

makani_c_library(
    name = "util_c",
    srcs = [
        "math.c",
        "utils.c",
    ],
    hdrs = [
        "fruit.h",
        "math.h",
        "utils.h",
    ],
)

makani_cc_library(
    name = "util_cc",
    srcs = [
        "utils.cc",
    ],
    hdrs = [
        "fruit.h",
        "utils.h",
    ],
)

py_wrap_c(
    name = "pywrap_lab_c",
    srcs = ["fruit.h"],
    c_deps = [
        ":util_c",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_all",
    ],
    # If the desired module name is the same as the name of the first swig
    # interface file (i.e., `miss_rate` in this case), then the `module_name`
    # parameter can be omitted here.
    # This produces `_<module_name>.so` and `<module_name>.py` as the output
    # of a py_library named `module_name`. By adding `module_name` to `deps`,
    # another rule can use `module_name.py` as a Python module with the wrapped
    # C/C++ functions.
    module_name = "lab_c",
    swig_interfaces = [
        "miss_rate.i",
        "percent.i",
    ],
)

py_wrap_cc(
    name = "pywrap_lab_cc",
    srcs = [
        "math.cc",
        "math.h",
    ],
    c_deps = [
        ":util_cc",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/network:aio_node",
        "//avionics/network:cvt_all",
    ],
    module_name = "lab_cc",
    swig_interfaces = [
        "portion_a.i",
        "portion_b.i",
    ],
)

py_wrap_c(
    name = "pywrap_array",
    srcs = [
        "array.c",
        "array.h",
        "fruit.h",
    ],
    c_deps = [
        "//third_party/swig_lib:numpy_headers",
    ],
    py_deps = [
        requirement("numpy"),
    ],
    swig_interfaces = [
        "array.i",
        "//third_party/swig_lib:numpy.i",
    ],
)

py_test(
    name = "swig_rules_test",
    srcs = ["swig_rules_test.py"],
    deps = [
        ":pywrap_array",
        ":pywrap_lab_c",
        ":pywrap_lab_cc",
    ],
)
