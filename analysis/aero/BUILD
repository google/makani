package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:c_rules.bzl", "makani_cc_binary")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library", "py_test")

py_binary(
    name = "plot_aero_blend",
    srcs = ["plot_aero_blend.py"],
    deps = [
        "//lib/python:flag_types",
        "//sim/physics:pywrap_physics",
        "//system:h2py_labels",
        requirement("matplotlib"),
        # The matplotlib Qt backend requires PyQt4, PyQt5, or PySide.  For
        # python3, where PySide is no longer supported, switch this to one
        # of the other Qt libraries.
        requirement("PySide"),
    ],
)

py_library(
    name = "load_database",
    srcs = ["load_database.py"],
    deps = [
        "//control:h2py_system_types",
        "//sim/physics:pywrap_physics",
    ],
)

py_library(
    name = "apparent_wind_util",
    srcs = ["apparent_wind_util.py"],
    deps = ["//analysis/control:geometry"],
)

py_test(
    name = "apparent_wind_util_test",
    srcs = ["apparent_wind_util_test.py"],
    deps = [":apparent_wind_util"],
)

py_binary(
    name = "plot_aero_database",
    srcs = ["plot_aero_database.py"],
    data = [
        "//database:local_databases",
    ],
    deps = [
        ":load_database",
        "//control:h2py_system_types",
        "//lib/python:flag_types",
        "//sim/physics:pywrap_physics",
        requirement("matplotlib"),
    ],
)

py_binary(
    name = "avl_mat_to_json",
    srcs = ["avl_mat_to_json.py"],
    deps = [
        "//control:h2py_system_types",
        "//lib/python:dict_util",
    ],
)

filegroup(
    name = "airfoils",
    srcs = glob(["airfoils/*"]),
)

makani_cc_binary(
    name = "aero_lookup",
    srcs = [
        "aero_lookup.cc",
    ],
    data = [
        "//database:local_databases",
    ],
    deps = [
        "//common:runfiles_dir",
        "//control:common",
        "//control:control_util",
        "//control:system_params",
        "//control:system_types",
        "//lib/json_load:load_params",
        "//sim:sim_params",
        "//sim:sim_types",
        "//sim/math",
        "//sim/physics",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_binary(
    name = "rotor_3d_lookup",
    srcs = [
        "rotor_3d_lookup.cc",
    ],
    data = [
        "//database:local_databases",
    ],
    deps = [
        "//common:macros",
        "//common:runfiles_dir",
        "//common/c_math",
        "//control:common",
        "//control:control_util",
        "//control:system_params",
        "//control:system_types",
        "//lib/json_load:load_params",
        "//sim:sim_params",
        "//sim:sim_types",
        "//sim/math",
        "//sim/physics",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

py_library(
    name = "airfoil",
    srcs = ["airfoil.py"],
    data = glob([
        "airfoils/*.json",
        "hover_model/airfoils/*.json",
    ]),
    deps = [
        requirement("numpy"),
        requirement("scipy"),
    ],
)

py_test(
    name = "airfoil_test",
    srcs = ["airfoil_test.py"],
    deps = [":airfoil"],
)

py_library(
    name = "wake_model",
    srcs = ["hover_model/wake_model.py"],
    deps = [
        requirement("numpy"),
        requirement("scipy"),
    ],
)

py_test(
    name = "wake_model_test",
    srcs = ["hover_model/wake_model_test.py"],
    deps = [":wake_model"],
)

py_library(
    name = "hover_model",
    srcs = ["hover_model/hover_model.py"],
    deps = [
        ":airfoil",
        ":wake_model",
        "//analysis/aero:apparent_wind_util",
        "//analysis/aero/avl:avl_reader",
        "//config:mconfig",
        "//lib/python:build_info",
        "//lib/python:c_helpers",
    ],
)

py_test(
    name = "hover_model_test",
    srcs = ["hover_model/hover_model_test.py"],
    deps = [
        ":hover_model",
        "//control:h2py_system_types",
    ],
)

py_library(
    name = "aero_ssam",
    srcs = ["aero_ssam.py"],
    deps = [":hover_model"],
)

py_test(
    name = "aero_ssam_test",
    srcs = ["aero_ssam_test.py"],
    deps = [":aero_ssam"],
)

py_binary(
    name = "generate_hover_database",
    srcs = ["hover_model/generate_hover_database.py"],
    deps = [
        ":hover_model",
        "//lib/python:dict_util",
        "//lib/python:flag_types",
    ],
)

py_binary(
    name = "plot_wake_model",
    srcs = ["hover_model/plot_wake_model.py"],
    deps = [
        ":hover_model",
        "//lib/python:flag_types",
        requirement("matplotlib"),
    ],
)

py_binary(
    name = "plot_rotor_database",
    srcs = ["plot_rotor_database.py"],
    deps = [
        "//analysis/aero:load_database",
        "//config:mconfig",
        "//control:h2py_system_types",
        requirement("matplotlib"),
    ],
)
