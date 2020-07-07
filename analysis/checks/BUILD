package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_library", "py_test")

py_library(
    name = "check_range",
    srcs = ["check_range.py"],
)

py_library(
    name = "log_util",
    srcs = [
        "iter_files.py",
        "log_util.py",
    ],
    deps = [
        "check_range",
        "//lib/python:gsutil",
        "//lib/python/batch_sim:gcloud_util",
    ],
)

py_library(
    name = "common",
    srcs = [
        "avionics_util.py",
        "base_check.py",
        "gradebook.py",
        "gradebook_base_check.py",
        "score_check.py",
    ],
    deps = [
        "check_range",
        "log_util",
        "//avionics/network:network_util",
        "//control:h2py_control_params",
        "//control:h2py_system_params",
        "//gs/monitor2/apps/layout:webmonitor_layout_util",
        "//gs/monitor2/apps/receiver:aio_util",
        "//lib/python:ctype_util",
        "//lib/python:debug_util",
        "//lib/python:dict_util",
        "//lib/python:struct_tree",
        "//lib/python/batch_sim:scoring_functions",
        "//lib/python/h5_utils:numpy_utils",
    ],
)

py_library(
    name = "autocheck",
    srcs = ["autocheck.py"],
    deps = [
        ":common",
        ":log_util",
        "//lib/python:lru_cache",
    ],
)

py_test(
    name = "tests",
    srcs = ["tests.py"],
    deps = [
        ":common",
        "//avionics/common:h2py_motor_thermal_types",
        "//gs/monitor2/apps/layout:webmonitor_layout_util",
        "//gs/monitor2/apps/plugins/layouts:autocheck_gradebook",
        "//gs/monitor2/apps/receiver:receiver_test_util",
        "//gs/monitor2/project:webmonitor_settings",
        "//lib/python:struct_tree",
        "//lib/python/h5_utils",
        requirement("jsmin"),
        requirement("mock"),
        requirement("numpy"),
    ],
)
