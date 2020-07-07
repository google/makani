package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library", "py_test")

# For json_load_test.
exports_files(["common/sim/sim_params.py"])

# The "light" versions of mconfig and write_params are needed to break a
# dependency cycle. Since //analysis/control:simple_aero depends on
# //control:system_params, the full mconfig and write_params cannot be used to
# generate system_params.c.
#
# If we ever need to break another cycle, we should probably make a metarule
# that customizes versions of these targets for each param genrule.
py_library(
    name = "mconfig_light",
    srcs = [
        "mconfig.py",
        "overrides_util.py",
        "physics_util.py",
        "sensor_util.py",
    ] + glob([
        "base_station/**/*.py",
        "common/**/*.py",
        "m600/**/*.py",
        "oktoberkite/**/*.py",
        "powertrain/**/*.py",
        "prop/**/*.py",
    ]),
    data = [
        "//avionics/loadcell/firmware:calib_params.yaml",
        "//avionics/servo/firmware:config_params.yaml",
    ],
    deps = [
        "//analysis/control:geometry",
        "//analysis/control:optimize",
        "//avionics/common:gps_receiver_py",
        "//avionics/common:h2py_network_config",
        "//avionics/common:h2py_plc_messages",
        "//avionics/firmware/params:param_util",
        "//avionics/network:h2py_aio_labels",
        "//control:h2py_control_types",
        "//control:h2py_sensor_types",
        "//control:h2py_system_types",
        "//gs/monitor:h2py_monitor_types",
        "//lib/python:c_helpers",
        "//lib/python:dict_util",
        "//lib/python:lru_cache",
        "//lib/python:units",
        "//lib/python:wing_flag",
        "//lib/python/autogen:autogen_util",
        "//sim:h2py_sim_types",
        "//system:h2py_labels",
        "//system:h2py_labels_util",
        requirement("cvxopt"),
        requirement("numpy"),
        requirement("python-gflags"),
        requirement("scipy"),
    ],
)

py_binary(
    name = "write_params_light",
    srcs = ["write_params.py"],
    main = "write_params.py",
    deps = [
        ":mconfig_light",
        "//control:h2py_system_types",
    ],
)

py_library(
    name = "mconfig",
    deps = [
        ":mconfig_light",
        "//analysis/control:simple_aero",
    ],
)

py_binary(
    name = "write_params",
    srcs = ["write_params.py"],
    deps = [
        ":mconfig",
        "//control:h2py_system_types",
    ],
)

py_test(
    name = "mconfig_test",
    srcs = ["mconfig_test.py"],
    deps = [
        ":mconfig",
        "//lib/python:test_util",
    ],
)

py_test(
    name = "overrides_util_test",
    srcs = ["overrides_util_test.py"],
    deps = [":mconfig"],
)
