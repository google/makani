package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library", "py_test")

py_library(
    name = "algorithms",
    srcs = ["algorithms.py"],
)

py_library(
    name = "build_info",
    srcs = ["build_info.py"],
    deps = ["//avionics/common:h2py_build_info"],
)

py_library(
    name = "ctype_util",
    srcs = ["ctype_util.py"],
    srcs_version = "PY2AND3",
    deps = [
        requirement("ctypeslib2"),
    ],
)

py_library(
    name = "c_helpers",
    srcs = ["c_helpers.py"],
    srcs_version = "PY2ONLY",
    deps = [
        "//lib/python/autogen:autogen_util",
        requirement("ctypeslib2"),
    ],
)

py_library(
    name = "debug_util",
    srcs = ["debug_util.py"],
)

py_library(
    name = "dict_util",
    srcs = ["dict_util.py"],
)

py_library(
    name = "environ_util",
    srcs = ["environ_util.py"],
    deps = [":git_util"],
)

py_library(
    name = "flag_types",
    srcs = ["flag_types.py"],
    deps = [
        requirement("numpy"),
        requirement("python-gflags"),
    ],
)

py_library(
    name = "git_util",
    srcs = ["git_util.py"],
    deps = [":os_util"],
)

py_library(
    name = "gsutil",
    srcs = ["gsutil.py"],
)

py_library(
    name = "gsutil_fakes",
    srcs = ["gsutil_fakes.py"],
    deps = [
        "//lib/python/batch_sim:gcloud_util",
        "//lib/python/batch_sim:test_util",
    ],
)

py_library(
    name = "h5log_loader",
    srcs = ["h5log_loader.py"],
)

py_library(
    name = "import_util",
    srcs = ["import_util.py"],
)

py_library(
    name = "ipython_completer",
    srcs = ["ipython_completer.py"],
)

py_library(
    name = "json_util",
    srcs = ["json_util.py"],
)

py_library(
    name = "lru_cache",
    srcs = ["lru_cache.py"],
)

# TODO(b/141989547): Split shell_interfaces from os_util.
py_library(
    name = "os_util",
    srcs = [
        "os_util.py",
        "shell_interfaces.py",
    ],
    deps = [
        requirement("python-gflags"),
    ],
)

py_library(
    name = "string_util",
    srcs = ["string_util.py"],
)

py_library(
    name = "struct_tree",
    srcs = ["struct_tree.py"],
    deps = [
        ":ctype_util",
        "//lib/python/h5_utils",
        "//lib/python/h5_utils:numpy_utils",
    ],
)

py_library(
    name = "test_util",
    srcs = ["test_util.py"],
    deps = [
        requirement("h5py"),
        requirement("mock"),
        requirement("numpy"),
        requirement("python-gflags"),
    ],
)

py_library(
    name = "turbsim_util",
    srcs = ["turbsim_util.py"],
)

py_library(
    name = "units",
    srcs = ["units.py"],
)

py_library(
    name = "wing_flag",
    srcs = ["wing_flag.py"],
    deps = [
        ":c_helpers",
        "//control:h2py_system_types",
        requirement("python-gflags"),
    ],
)

py_test(
    name = "algorithms_test",
    srcs = ["algorithms_test.py"],
    deps = [":algorithms"],
)

py_test(
    name = "build_info_test",
    srcs = ["build_info_test.py"],
    deps = [":build_info"],
)

py_test(
    name = "c_helpers_test",
    srcs = ["c_helpers_test.py"],
    deps = [
        ":c_helpers",
        "//avionics/common:h2py_pack_avionics_messages",
        "//common/c_math:h2py_filter",
        "//common/c_math:h2py_util",
    ],
)

py_test(
    name = "dict_util_test",
    srcs = ["dict_util_test.py"],
    deps = [":dict_util"],
)

py_test(
    name = "import_util_test",
    srcs = ["import_util_test.py"],
    deps = [":import_util"],
)

py_test(
    name = "flag_types_test",
    srcs = ["flag_types_test.py"],
    deps = [":flag_types"],
)

py_test(
    name = "lru_cache_test",
    srcs = ["lru_cache_test.py"],
    deps = [":lru_cache"],
)

py_test(
    name = "os_util_test",
    srcs = ["os_util_test.py"],
    deps = [
        ":os_util",
        ":test_util",
    ],
)

py_binary(
    name = "shell_interfaces_test_manual",
    srcs = ["shell_interfaces_test_manual.py"],
    deps = [
        ":os_util",
        ":test_util",
    ],
)

py_test(
    name = "string_util_test",
    srcs = ["string_util_test.py"],
    deps = [":string_util"],
)

py_test(
    name = "struct_tree_test",
    srcs = ["struct_tree_test.py"],
    deps = [
        ":ctype_util",
        ":struct_tree",
        "//gs/monitor2/apps/receiver:receiver_test_util",
        "//lib/python/h5_utils:h5_io_test",
    ],
)

py_test(
    name = "test_util_test",
    srcs = ["test_util_test.py"],
    deps = [":test_util"],
)
