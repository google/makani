package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_library",
    "makani_cc_test",
)
load("//lib/bazel:py_rules.bzl", "py_binary", "py_test")
load("//lib/bazel:swig_rules.bzl", "py_wrap_cc")

makani_c_library(
    name = "aero_types",
    hdrs = ["aero_types.h"],
)

py_wrap_cc(
    name = "pywrap_physics",
    c_deps = [
        ":physics",
        "//common/c_math",
        "//control:system_params",
    ],
    swig_interfaces = ["physics.i"],
)

py_test(
    name = "pywrap_physics_test",
    srcs = ["pywrap_physics_test.py"],
    deps = [
        ":pywrap_physics",
        "//control:h2py_system_types",
    ],
)

makani_cc_library(
    name = "physics",
    srcs = [
        "aero.cc",
        "avl_aero_database.cc",
        "contactor.cc",
        "delta_coeff_aero_database.cc",
        "dvl_aero_database.cc",
        "motors.cc",
        "reference_frame.cc",
        "rotor_database.cc",
        "rotor_database_3d.cc",
        "wind.cc",
    ],
    hdrs = [
        "aero.h",
        "avl_aero_database.h",
        "contactor.h",
        "delta_coeff_aero_database.h",
        "dvl_aero_database.h",
        "motors.h",
        "reference_frame.h",
        "rotor_database.h",
        "rotor_database_3d.h",
        "wind.h",
    ],
    data = [
        "//database:m600_avl_database",
        "//database:oktoberkite_avl_database",
        "//database:oktoberkite_high_aoa_databases",
        "//database:sn03_high_aoa_databases",
    ],
    deps = [
        ":aero_frame",
        ":aero_types",
        "//common:macros",
        "//common:runfiles_dir",
        "//common/c_math",
        "//control:system_types",
        "//lib/json_load:json_array_loader",
        "//lib/json_load:json_load_or_die",
        "//sim:sim_params",
        "//sim/math",
        "@usr_lib//:glog",
        "@usr_lib//:hdf5",
    ],
)

makani_c_library(
    name = "wind_frame",
    srcs = ["wind_frame.c"],
    hdrs = ["wind_frame.h"],
    deps = ["//common/c_math"],
)

makani_cc_library(
    name = "ground_frame",
    srcs = ["ground_frame.cc"],
    hdrs = ["ground_frame.h"],
    deps = [
        ":physics",
        "//common:macros",
        "//common/c_math",
        "//control:control_util",
        "//sim:sim_types",
    ],
)

makani_cc_test(
    name = "contactor_test",
    srcs = ["contactor_test.cc"],
    deps = [":physics"],
)

makani_cc_library(
    name = "aero_frame",
    srcs = ["aero_frame.cc"],
    hdrs = ["aero_frame.h"],
    deps = ["//common/c_math"],
)

makani_cc_test(
    name = "aero_frame_test",
    srcs = ["aero_frame_test.cc"],
    deps = [
        ":aero_frame",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "aero_test",
    size = "large",
    srcs = ["aero_test.cc"],
    deps = [
        ":physics",
        "//control:system_params",
        "//lib/util:test_util",
        "@usr_lib//:glog",
    ],
)

makani_cc_test(
    name = "reference_frame_test",
    srcs = ["reference_frame_test.cc"],
    deps = [
        ":physics",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "wind_frame_test",
    srcs = ["wind_frame_test.cc"],
    deps = [
        ":wind_frame",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "wind_test",
    srcs = ["wind_test.cc"],
    data = [
        "test_data/example_wind.h5",
    ],
    deps = [
        ":physics",
        "//lib/util:test_util",
    ],
)

py_binary(
    name = "generate_example_wind",
    srcs = ["test_data/generate_example_wind.py"],
    deps = [
        requirement("h5py"),
        requirement("numpy"),
        requirement("python-gflags"),
    ],
)

# Produces a dummy wind database for testing.
genrule(
    name = "example_wind_genrule",
    outs = ["test_data/example_wind.h5"],
    cmd = "$(location :generate_example_wind) --output_file=$@",
    tools = [":generate_example_wind"],
)

h2py_library(
    name = "h2py_wind_frame",
    function_deps = [":wind_frame"],
    header = "wind_frame.h",
)
