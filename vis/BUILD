package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load(
    "//lib/bazel:c_rules.bzl",
    "makani_c_library",
    "makani_cc_binary",
)
load("//lib/bazel:py_rules.bzl", "py_binary")

makani_c_library(
    name = "vislib",
    srcs = glob([
        "*.c",
        "*.h",
    ]) + [
        "m600_model.h",
        "oktoberkite_model.h",
        "gs02_detwist.h",
        "gs02_drum.h",
        "gs02_levelwind_arm.h",
        "gs02_levelwind_cassette.h",
        "gs02_platform.h",
        "gs02_termination.h",
        "gs02_tower.h",
        "gs02_yoke.h",
        "spar_buoy.h",
    ],
    deps = [
        "//avionics/common:tether_message",
        "//avionics/linux:aio",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
        "//common:macros",
        "//common/c_math",
        "//control:control_params",
        "//control:control_types",
        "//control:control_util",
        "//control:cvt_control_telemetry",
        "//control:system_params",
        "//control:system_types",
        "//gs/monitor:monitor_params",
        "//sim:cvt_sim_telemetry",
        "//sim:sim_params",
        "//sim:sim_types",
    ],
)

makani_cc_binary(
    name = "vis",
    srcs = ["vis_main.cc"],
    linkopts = [
        "-lglut",
        "-lGLU",
        "-lGL",
        "-lrt",
    ],
    deps = [
        "//avionics/linux:clock",
        "//lib/json_load:load_params",
        "//vis:vislib",
        "@usr_lib//:gflags",
        "@usr_lib//:glog",
    ],
)

py_binary(
    name = "generate_model",
    srcs = ["generate_model.py"],
    deps = [
        "//lib/python:wing_flag",
        requirement("python-gflags"),
    ],
)

genrule(
    name = "m600_model_genrule",
    srcs = ["m600_model.txt"],
    outs = ["m600_model.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--model_name=m600",
        "--input_file=$(location m600_model.txt)",
        "--output_file=$(location m600_model.h)",
    ]),
    tools = [":generate_model"],
)

genrule(
    name = "oktoberkite_model_genrule",
    srcs = ["oktoberkite_model.txt"],
    outs = ["oktoberkite_model.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--model_name=oktoberkite",
        "--input_file=$(location oktoberkite_model.txt)",
        "--output_file=$(location oktoberkite_model.h)",
    ]),
    tools = [":generate_model"],
)

genrule(
    name = "gs02_platform_genrule",
    srcs = ["gs02_platform.txt.gz"],
    outs = ["gs02_platform.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--gunzip",
        "--model_name=gs02_platform",
        "--input_file=$(location gs02_platform.txt.gz)",
        "--output_file=$(location gs02_platform.h)",
    ]),
    tools = [":generate_model"],
)

genrule(
    name = "gs02_drum_genrule",
    srcs = ["gs02_drum.txt.gz"],
    outs = ["gs02_drum.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--gunzip",
        "--model_name=gs02_drum",
        "--input_file=$(location gs02_drum.txt.gz)",
        "--output_file=$(location gs02_drum.h)",
    ]),
    tools = [":generate_model"],
)

genrule(
    name = "gs02_levelwind_arm_genrule",
    srcs = ["gs02_levelwind_arm.txt.gz"],
    outs = ["gs02_levelwind_arm.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--gunzip",
        "--model_name=gs02_levelwind_arm",
        "--input_file=$(location gs02_levelwind_arm.txt.gz)",
        "--output_file=$(location gs02_levelwind_arm.h)",
    ]),
    tools = [":generate_model"],
)

genrule(
    name = "gs02_levelwind_cassette_genrule",
    srcs = ["gs02_levelwind_cassette.txt.gz"],
    outs = ["gs02_levelwind_cassette.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--gunzip",
        "--model_name=gs02_levelwind_cassette",
        "--input_file=$(location gs02_levelwind_cassette.txt.gz)",
        "--output_file=$(location gs02_levelwind_cassette.h)",
    ]),
    tools = [":generate_model"],
)

genrule(
    name = "gs02_termination_genrule",
    srcs = ["gs02_termination.txt.gz"],
    outs = ["gs02_termination.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--gunzip",
        "--model_name=gs02_termination",
        "--input_file=$(location gs02_termination.txt.gz)",
        "--output_file=$(location gs02_termination.h)",
    ]),
    tools = [":generate_model"],
)

genrule(
    name = "gs02_yoke_genrule",
    srcs = ["gs02_yoke.txt.gz"],
    outs = ["gs02_yoke.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--gunzip",
        "--model_name=gs02_yoke",
        "--input_file=$(location gs02_yoke.txt.gz)",
        "--output_file=$(location gs02_yoke.h)",
    ]),
    tools = [":generate_model"],
)

genrule(
    name = "gs02_detwist_genrule",
    srcs = ["gs02_detwist.txt.gz"],
    outs = ["gs02_detwist.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--gunzip",
        "--model_name=gs02_detwist",
        "--input_file=$(location gs02_detwist.txt.gz)",
        "--output_file=$(location gs02_detwist.h)",
    ]),
    tools = [":generate_model"],
)

genrule(
    name = "gs02_tower_genrule",
    srcs = ["gs02_tower.txt.gz"],
    outs = ["gs02_tower.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--gunzip",
        "--model_name=gs02_tower",
        "--input_file=$(location gs02_tower.txt.gz)",
        "--output_file=$(location gs02_tower.h)",
    ]),
    tools = [":generate_model"],
)

genrule(
    name = "spar_buoy_genrule",
    srcs = ["spar_buoy.txt.gz"],
    outs = ["spar_buoy.h"],
    cmd = " ".join([
        "$(location :generate_model)",
        "--gunzip",
        "--model_name=spar_buoy",
        "--input_file=$(location spar_buoy.txt.gz)",
        "--output_file=$(location spar_buoy.h)",
    ]),
    tools = [":generate_model"],
)
