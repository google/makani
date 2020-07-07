package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library", "py_test")

py_library(
    name = "actuator_util",
    srcs = [
        "actuator_util.py",
    ],
    deps = [
        "//control:h2py_actuator_util",
        "//lib/python:ctype_util",
        "//lib/python/autogen:autogen_util",
    ],
)

py_test(
    name = "actuator_util_test",
    srcs = ["actuator_util_test.py"],
    deps = [
        ":actuator_util",
        "//config:mconfig",
    ],
)

py_library(
    name = "simple_aero",
    srcs = [
        "simple_aero.py",
    ] + glob([
        "common/**/*.py",
        "m600/**/*.py",
    ]),
    data = [
        "//database:m600_avl_database",
        "//database:oktoberkite_avl_database",
        "//database:rotor_databases",
    ],
    deps = [
        "//analysis/aero:apparent_wind_util",
        "//analysis/aero:load_database",
        "//avionics/network:h2py_aio_labels",
        "//control:h2py_simple_aero",
    ],
)

py_test(
    name = "simple_aero_test",
    srcs = ["simple_aero_test.py"],
    deps = [
        ":simple_aero",
        "//config:mconfig",
    ],
)

py_binary(
    name = "plot_simple_rotor_model",
    srcs = ["plot_simple_rotor_model.py"],
    deps = [
        ":simple_aero",
        "//analysis/aero:load_database",
        "//config:mconfig",
        "//control:h2py_system_types",
    ],
)

py_library(
    name = "geometry",
    srcs = ["geometry.py"],
    deps = [
        ":type_util",
        "//lib/python:c_helpers",
        "//sim:h2py_sim_types",
    ],
)

py_test(
    name = "geometry_test",
    srcs = ["geometry_test.py"],
    deps = [
        ":geometry",
        requirement("scipy"),
    ],
)

py_library(
    name = "dynamics",
    srcs = ["dynamics.py"],
    data = [
        "//database:m600_avl_database",
        "//database:oktoberkite_avl_database",
        "//database:oktoberkite_high_aoa_databases",
        "//database:sn03_high_aoa_databases",
    ],
    deps = [
        ":actuator_util",
        ":catenary",
        ":geometry",
        ":type_util",
        "//sim:h2py_sim_types",
        "//sim/physics:pywrap_physics",
    ],
)

py_test(
    name = "dynamics_test",
    srcs = ["dynamics_test.py"],
    deps = [
        ":dynamics",
        "//config:mconfig",
    ],
)

py_binary(
    name = "dynamics_comparison",
    srcs = ["dynamics_comparison.py"],
    deps = [
        ":dynamics",
        ":type_util",
        "//config:mconfig",
    ],
)

py_binary(
    name = "trans_in",
    srcs = ["trans_in.py"],
    deps = [
        ":dynamics",
        ":geometry",
        ":systems",
        ":type_util",
        ":write_controllers",
        "//config:mconfig",
        "//control:h2py_system_types",
        "//lib/python:c_helpers",
        requirement("control"),
        requirement("slycot"),
    ],
)

py_binary(
    name = "crosswind",
    srcs = ["crosswind.py"],
    deps = [
        ":dynamics",
        ":type_util",
        ":write_controllers",
        "//config:mconfig",
        "//lib/python:c_helpers",
        "//lib/python:wing_flag",
        requirement("control"),
        requirement("slycot"),
    ],
)

py_binary(
    name = "flap_limits",
    srcs = ["flap_limits.py"],
    data = [
        "//avionics/servo/firmware:config_params.yaml",
    ],
    deps = [
        "//avionics/bootloader:system_config",
        "//avionics/firmware/params:param_util",
        "//config:mconfig",
    ],
)

py_test(
    name = "flap_limits_test",
    srcs = ["flap_limits_test.py"],
    deps = [
        ":flap_limits",
    ],
)

py_binary(
    name = "manual",
    srcs = ["manual.py"],
    deps = [
        ":dynamics",
        ":type_util",
        "//config:mconfig",
    ],
)

py_test(
    name = "trans_in_test",
    srcs = ["trans_in_test.py"],
    deps = [
        ":trans_in",
        "//config:mconfig",
    ],
)

py_library(
    name = "catenary",
    srcs = ["catenary.py"],
    deps = [
        requirement("numpy"),
        requirement("scipy"),
    ],
)

py_test(
    name = "catenary_test",
    srcs = ["catenary_test.py"],
    deps = [
        ":catenary",
    ],
)

py_library(
    name = "type_util",
    srcs = ["type_util.py"],
    deps = [
        requirement("numpy"),
    ],
)

py_test(
    name = "type_util_test",
    srcs = ["type_util_test.py"],
    deps = [
        ":type_util",
    ],
)

py_library(
    name = "optimize",
    srcs = ["optimize.py"],
    deps = [
        requirement("cvxopt"),
    ],
)

py_test(
    name = "optimize_test",
    srcs = ["optimize_test.py"],
    deps = [
        ":geometry",
        ":optimize",
    ],
)

py_library(
    name = "systems",
    srcs = ["systems.py"],
    deps = [
        requirement("numpy"),
    ],
)

py_test(
    name = "systems_test",
    srcs = ["systems_test.py"],
    deps = [":systems"],
)

py_library(
    name = "write_controllers",
    srcs = ["write_controllers.py"],
    deps = [
        "//control:h2py_system_types",
        "//lib/python:c_helpers",
    ],
)
