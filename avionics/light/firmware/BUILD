package(default_visibility = ["//visibility:private"])

load("//avionics/firmware:build_rules.bzl", "tms570_application")
load("//avionics/firmware/params:build_rules.bzl", "gen_param_file")

# TMS570 applications.

tms570_application(
    name = "light_application.elf",
    srcs = [],
    deps = [
        "//avionics/fc/firmware:fc_library",
        "//avionics/network:cvt_light",
    ],
)

# Parameter files.
# TODO: These rules should probably go away once we have proper MES
# integration.

gen_param_file(
    src = "config_params.yaml",
    out = "port_light_config_params.bin",
    key = "port",
)

gen_param_file(
    src = "config_params.yaml",
    out = "stbd_light_config_params.bin",
    key = "stbd",
)

gen_param_file(
    src = "config_params.yaml",
    out = "tail_light_config_params.bin",
    key = "tail",
)
