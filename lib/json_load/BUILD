package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_library",
    "makani_cc_test",
)
load("//lib/bazel:py_rules.bzl", "py_binary")

py_binary(
    name = "json_load_generate",
    srcs = [
        "json_load.py",
        "json_load_generate.py",
    ],
    main = "json_load_generate.py",
    deps = [
        "//control:h2py_control_types",
        "//gs/monitor:h2py_monitor_types",
        "//lib/python/autogen:autogen_util",
        "//sim:h2py_sim_types",
        requirement("python-gflags"),
    ],
)

genrule(
    name = "json_load_genrule",
    outs = [
        "json_load.c",
        "json_load.h",
        "json_load_priv.h",
    ],
    cmd = ("$(location :json_load_generate) --autogen_root=$(GENDIR)" +
           " ControlParams MonitorParams SimParams SystemParams"),
    tools = [":json_load_generate"],
)

makani_c_library(
    name = "json_load",
    srcs = [
        "json_load.c",
        "json_load_basic.c",
        "json_load_priv.h",
    ],
    hdrs = [
        "json_load.h",
        "json_load_basic.h",
    ],
    deps = [
        "//avionics/common:gps_types",
        "//avionics/common:novatel_types",
        "//avionics/network:aio_node",
        "//common/c_math",
        "//control:control_types",
        "//control:sensor_types",
        "//control:system_types",
        "//control/crosswind:crosswind_types",
        "//control/estimator:estimator_types",
        "//control/fault_detection:fault_detection_types",
        "//control/hover:hover_types",
        "//control/manual:manual_types",
        "//control/trans_in:trans_in_types",
        "//gs/monitor:monitor_types",
        "//sim:sim_types",
        "//system:labels",
        "@usr_include//:jansson",
    ],
)

makani_cc_library(
    name = "json_array_loader",
    srcs = [],
    hdrs = ["json_array_loader.h"],
    linkopts = ["-ljansson"],
    deps = [
        ":json_load_or_die",
        "@usr_include//:jansson",
        "@usr_lib//:glog",
    ],
)

makani_cc_library(
    name = "json_load_or_die",
    srcs = ["json_load_or_die.cc"],
    hdrs = ["json_load_or_die.h"],
    deps = [
        ":json_load",
        "@usr_include//:jansson",
        "@usr_lib//:glog",
    ],
)

makani_cc_library(
    name = "load_params",
    srcs = ["load_params.cc"],
    hdrs = ["load_params.h"],
    deps = [
        ":json_load",
        ":json_load_or_die",
        "//control:control_types",
        "//control:system_types",
        "//gs/monitor:monitor_types",
        "//sim:sim_types",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

makani_cc_test(
    name = "json_load_basic_test",
    srcs = ["json_load_basic_test.cc"],
    copts = ["-Wno-effc++"],
    deps = [":json_load"],
)

# Generates sim_params.json for use in json_load_test.
genrule(
    name = "sim_params_genrule",
    srcs = ["//config:common/sim/sim_params.py"],
    outs = ["test_data/sim_params.json"],
    cmd = " ".join([
        "$(location //config:write_params)",
        "--type=json",
        "--input_file=$<",
        "--output_file=$@",
    ]),
    tools = ["//config:write_params"],
)

makani_cc_test(
    name = "json_load_test",
    srcs = ["json_load_test.cc"],
    data = ["test_data/sim_params.json"],
    deps = [
        ":json_load",
        "//common:macros",
        "//lib/util:test_util",
        "//sim:sim_params",
    ],
)

makani_cc_test(
    name = "json_load_or_die_test",
    srcs = ["json_load_or_die_test.cc"],
    copts = ["-Wno-effc++"],
    data = ["test_data/simple.json"],
    deps = [
        ":json_load_or_die",
        "//common:macros",
        "//lib/util:test_util",
    ],
)
