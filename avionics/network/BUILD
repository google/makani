package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load(
    "//avionics/network:build_rules.bzl",
    "makani_cvt_genrule",
    "makani_network_genrule",
)
load("//lib/bazel:autogen_rules.bzl", "h2py_library")
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_binary",
    "makani_cc_test",
)
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library", "py_test")

# C libraries.

makani_c_library(
    name = "aio_latency",
    srcs = ["aio_latency.c"],
    hdrs = ["aio_latency.h"],
    deps = [
        "//avionics/common:cvt_avionics_messages",
        "//avionics/firmware/network:aio",
        "//avionics/network:aio_node",
    ],
)

makani_c_library(
    name = "aio_labels",
    srcs = ["aio_labels.c"],
    hdrs = ["aio_labels.h"],
    deps = [":aio_node"],
)

makani_c_library(
    name = "aio_node",
    srcs = ["aio_node.c"],
    hdrs = ["aio_node.h"],
    deps = ["//common:macros"],
)

makani_c_library(
    name = "aio_node_to_ip_address",
    srcs = ["aio_node_to_ip_address.c"],
    hdrs = ["aio_node_to_ip_address.h"],
    deps = [
        ":aio_node",
        "//avionics/common:network_config",
    ],
)

makani_c_library(
    name = "eop_message_type",
    srcs = ["eop_message_type.c"],
    hdrs = ["eop_message_type.h"],
)

makani_c_library(
    name = "message_info",
    srcs = ["message_info.c"],
    hdrs = ["message_info.h"],
    deps = [
        ":message_type",
        "//avionics/common:aio_header",
        "//avionics/common:avionics_messages",
        "//avionics/common:pack_avionics_messages",
        "//control:control_telemetry",
        "//control:ground_telemetry",
        "//control:pack_control_telemetry",
        "//control:pack_ground_telemetry",
        "//sim:pack_sim_messages",
        "//sim:pack_sim_telemetry",
        "//sim:sim_messages",
        "//sim:sim_telemetry",
    ],
)

makani_c_library(
    name = "message_stats",
    srcs = ["message_stats.c"],
    hdrs = ["message_stats.h"],
    deps = [
        ":aio_node",
        ":message_type",
    ],
)

makani_c_library(
    name = "message_type",
    srcs = ["message_type.c"],
    hdrs = ["message_type.h"],
)

makani_c_library(
    name = "switch_config",
    srcs = [
        "switch_config.c",
        "switch_info.c",
    ],
    hdrs = [
        "switch_config.h",
        "switch_info.h",
    ],
    deps = [
        ":aio_node",
        ":switch_types",
        "//avionics/common:network_config",
    ],
)

makani_c_library(
    name = "routes",
    srcs = [
        "route_config.c",
        "routes.c",
    ],
    hdrs = [
        "message_type.h",
        "route_config.h",
        "routes.h",
    ],
    deps = [
        ":aio_node",
        ":switch_config",
        ":switch_types",
        "//avionics/common:winch_messages",
    ],
)

makani_c_library(
    name = "switch_def",
    hdrs = ["switch_def.h"],
)

makani_c_library(
    name = "switch_types",
    hdrs = [
        "switch_types.h",
    ],
    deps = [
        ":switch_def",
        "//avionics/common:network_addresses",
        "//avionics/common:network_config",
    ],
)

makani_c_library(
    name = "switch_links",
    srcs = ["switch_links.c"],
    hdrs = ["switch_links.h"],
    deps = [
        ":aio_node",
    ],
)

makani_c_library(
    name = "winch_message_type",
    srcs = ["winch_message_type.c"],
    hdrs = ["winch_message_type.h"],
)

# Python libraries.

py_library(
    name = "aio_message",
    srcs = ["aio_message.py"],
    deps = [
        ":network_config",
        "//avionics/common:h2py_pack_avionics_messages",
        "//control:h2py_pack_control_telemetry",
        "//control:h2py_pack_ground_telemetry",
        "//sim:h2py_pack_sim_messages",
        "//sim:h2py_pack_sim_telemetry",
    ],
)

py_library(
    name = "network_util",
    srcs = ["network_util.py"],
    deps = [
        ":network_config",
    ],
)

py_library(
    name = "network_config",
    srcs = ["network_config.py"],
    data = ["network.yaml"],
    deps = [
        "//lib/python:string_util",
        requirement("python-gflags"),
        requirement("PyYAML"),
    ],
)

py_library(
    name = "node_locations",
    srcs = ["node_locations.py"],
    deps = [
        ":network_util",
    ],
)

# Python binaries.

py_binary(
    name = "aio_dump",
    srcs = ["aio_dump.py"],
    python_version = "PY2",
    tags = ["tms570_util"],
    deps = [
        "//avionics/common:aio",
        "//lib/python:ctype_util",
        requirement("python-gflags"),
    ],
)

py_binary(
    name = "aio_ping",
    srcs = ["aio_ping.py"],
    tags = ["tms570_util"],
    deps = ["//avionics/common:aio"],
)

py_binary(
    name = "aio_pygen",
    srcs = ["aio_pygen.py"],
    deps = [
        ":h2py_aio_node",
        ":h2py_message_type",
        "//avionics/common:aio",
    ],
)

py_binary(
    name = "generate_aio_labels",
    srcs = ["generate_aio_labels.py"],
    deps = [
        ":network_config",
    ],
)

py_binary(
    name = "generate_aio_node",
    srcs = ["generate_aio_node.py"],
    deps = [
        ":network_config",
        ":node_locations",
    ],
)

py_binary(
    name = "generate_aio_node_to_ip_address",
    srcs = ["generate_aio_node_to_ip_address.py"],
    deps = [":network_config"],
)

py_binary(
    name = "generate_cvt",
    srcs = ["generate_cvt.py"],
    deps = [
        ":aio_message",
        ":network_config",
    ],
)

py_binary(
    name = "generate_message_info",
    srcs = ["generate_message_info.py"],
    deps = [
        ":aio_message",
        "//lib/python:c_helpers",
    ],
)

py_binary(
    name = "generate_message_stats",
    srcs = ["generate_message_stats.py"],
    deps = [":network_config"],
)

py_binary(
    name = "generate_message_type",
    srcs = ["generate_message_type.py"],
    deps = [
        ":network_config",
    ],
)

py_binary(
    name = "generate_routes",
    srcs = ["generate_routes.py"],
    deps = [
        ":network_config",
        ":network_util",
    ],
)

py_binary(
    name = "generate_switch_def",
    srcs = ["generate_switch_def.py"],
    deps = [":network_config"],
)

py_binary(
    name = "generate_switch_info",
    srcs = ["generate_switch_info.py"],
    deps = [
        ":h2py_aio_node",
        ":h2py_eop_message_type",
        ":h2py_message_type",
        ":h2py_winch_message_type",
        ":network_config",
        ":network_util",
        "//avionics/common:h2py_network_config",
        "//lib/python:c_helpers",
    ],
)

py_binary(
    name = "generate_switch_links",
    srcs = ["generate_switch_links.py"],
    deps = [
        ":network_config",
        ":network_util",
        "//lib/python:c_helpers",
        "//lib/python:string_util",
    ],
)

py_binary(
    name = "network_bandwidth",
    srcs = ["network_bandwidth.py"],
    deps = [
        ":h2py_aio_node",
        ":h2py_message_type",
        ":network_config",
        ":network_util",
        ":node_locations",
        "//avionics/common:h2py_pack_aio_header",
        "//avionics/common:h2py_pack_avionics_messages",
        "//avionics/common:h2py_pack_eop_messages",
        "//avionics/common:h2py_pack_winch_messages",
        "//control:h2py_pack_control_telemetry",
        "//control:h2py_pack_ground_telemetry",
    ],
)

py_binary(
    name = "network_dashboard",
    srcs = ["network_dashboard.py"],
    data = [
        "network_dashboard.html",
        "network_dashboard_bar.html",
        "network_dashboard_gauge.html",
        "network_dashboard_pie.html",
    ],
    deps = [
        "//avionics/common:h2py_pack_avionics_messages",
        "//control:h2py_pack_control_telemetry",
        "//control:h2py_pack_ground_telemetry",
    ],
)

py_binary(
    name = "network_routes",
    srcs = ["network_routes.py"],
    deps = [
        ":network_config",
        ":network_util",
    ],
)

py_binary(
    name = "route_dump",
    srcs = ["route_dump.py"],
    tags = ["tms570_util"],
    deps = [
        "//avionics/common:aio",
        "//lib/python:ctype_util",
    ],
)

# C/C++ binaries.

makani_cc_binary(
    name = "aio_gen",
    srcs = ["aio_gen.cc"],
    archs = [
        "x86",
        "q7",
    ],
    deps = [
        ":message_info",
        ":message_stats",
        "//avionics/linux:aio",
        "//common:macros",
    ],
)

makani_cc_binary(
    name = "aio_tunnel",
    srcs = ["aio_tunnel.cc"],
    archs = ["x86"],
    deps = [
        "//avionics/linux:aio",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
        "@usr_lib//:snappy",
    ],
)

# H2PY rules.

# TODO: Deprecate these h2py rules for NetworkConfig.

h2py_library(
    name = "h2py_aio_labels",
    function_deps = [":aio_labels"],
    header = "aio_labels.h",
)

h2py_library(
    name = "h2py_aio_node",
    function_deps = [":aio_node"],
    header = "aio_node.h",
)

h2py_library(
    name = "h2py_aio_node_to_ip_address",
    function_deps = [":aio_node_to_ip_address"],
    header = "aio_node_to_ip_address.h",
)

h2py_library(
    name = "h2py_eop_message_type",
    header = "eop_message_type.h",
    header_deps = [":eop_message_type"],
)

h2py_library(
    name = "h2py_message_type",
    header = "message_type.h",
    header_deps = [":message_type"],
)

h2py_library(
    name = "h2py_winch_message_type",
    header = "winch_message_type.h",
    header_deps = [":winch_message_type"],
)

# Code generation rules.

makani_network_genrule(
    name = "aio_labels_genrule",
    outs = [
        "aio_labels.c",
        "aio_labels.h",
        "aio_labels_test.cc",
    ],
    tools = [":generate_aio_labels"],
)

makani_network_genrule(
    name = "aio_node_genrule",
    outs = [
        "aio_node.c",
        "aio_node.h",
    ],
    tools = [":generate_aio_node"],
)

makani_network_genrule(
    name = "aio_node_to_ip_address_genrule",
    outs = ["aio_node_to_ip_address.c"],
    tools = [":generate_aio_node_to_ip_address"],
)

makani_network_genrule(
    name = "cvt_genrule",
    outs = ["aio_cvt_config.h"],
    tools = [":generate_cvt"],
)

genrule(
    name = "message_info_genrule",
    outs = [
        "message_info.c",
        "message_info.h",
    ],
    cmd = " ".join([
        "$(location :generate_message_info)",
        "--autogen_root=$(GENDIR)",
    ]),
    tools = [":generate_message_info"],
)

makani_network_genrule(
    name = "message_stats_genrule",
    outs = [
        "message_stats.c",
        "message_stats.h",
    ],
    tools = [":generate_message_stats"],
)

makani_network_genrule(
    name = "message_type_genrule",
    outs = [
        "eop_message_type.c",
        "eop_message_type.h",
        "message_type.c",
        "message_type.h",
        "winch_message_type.c",
        "winch_message_type.h",
    ],
    tools = [":generate_message_type"],
)

makani_network_genrule(
    name = "routes_genrule",
    outs = [
        "routes.c",
        "routes.h",
    ],
    tools = [":generate_routes"],
)

makani_network_genrule(
    name = "switch_def_genrule",
    outs = ["switch_def.h"],
    tools = [":generate_switch_def"],
)

makani_network_genrule(
    name = "switch_info_genrule",
    outs = ["switch_info.c"],
    tools = [":generate_switch_info"],
)

makani_network_genrule(
    name = "switch_links_genrule",
    outs = [
        "switch_links.c",
        "switch_links.h",
    ],
    tools = [":generate_switch_links"],
)

# CVT code generation.

makani_cvt_genrule(
    name = "cvt_all",
    all_nodes = True,
)

makani_cvt_genrule(
    name = "cvt_tms570s",
    all_tms570s = True,
)

makani_cvt_genrule(
    name = "cvt_q7s",
    all_q7s = True,
)

[
    makani_cvt_genrule(
        name = "cvt_" + aio_label,
        aio_labels = aio_label,
    )
    for aio_label in [
        "batt",
        "controller",
        "core_switch",
        "drum",
        "dyno_motor",
        "flight_computer",
        "fmm",
        "gps",
        "ground_power_tms570",
        "joystick",
        "light",
        "loadcell_node",
        "motor",
        "mvlv",
        "platform",
        "plc_gs02",
        "plc_tophat",
        "recorder_tms570",
        "servo",
        "short_stack",
    ]
]

# C/C++ tests.

makani_cc_test(
    name = "aio_labels_test",
    srcs = ["aio_labels_test.cc"],
    deps = [":aio_labels"],
)

makani_cc_test(
    name = "network_test",
    srcs = ["network_test.cc"],
    deps = [":aio_node_to_ip_address"],
)

# Python tests.

py_test(
    name = "network_bandwidth_test",
    srcs = ["network_bandwidth_test.py"],
    deps = [
        ":network_bandwidth",
        ":network_routes",
    ],
)

py_test(
    name = "network_routes_test",
    srcs = ["network_routes_test.py"],
    deps = [":network_routes"],
)

py_test(
    name = "network_util_test",
    srcs = ["network_util_test.py"],
    data = ["test.yaml"],
    deps = [
        ":network_config",
        ":network_util",
    ],
)

py_test(
    name = "network_yaml_test",
    srcs = ["network_yaml_test.py"],
    deps = [
        ":network_config",
        ":network_util",
    ],
)
