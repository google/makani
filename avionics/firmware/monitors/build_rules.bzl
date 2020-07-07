"""This module contains monitor genrules."""

load("//lib/bazel:py_rules.bzl", "py_binary")

def _monitor_genrule(monitor_name, name, deps, **kwargs):
    py_binary(
        name = "generate_" + name,
        srcs = [
            "//avionics/firmware/monitors:generate_" + monitor_name +
            "_monitor.py",
            "//avionics/firmware/monitors:generate_monitor_base.py",
            name + ".py",
        ],
        main = "//avionics/firmware/monitors:generate_" + monitor_name +
               "_monitor.py",
        deps = deps + [
            "//avionics/firmware/monitors:generate_monitor_base",
        ],
    )

    native.genrule(
        name = name + "_genrule",
        srcs = [name + ".py"],
        outs = [
            name + "_types.c",
            name + "_types.h",
        ],
        tools = [":generate_" + name],
        cmd = " ".join([
            "$(location :generate_" + name + ")",
            "--autogen_root=$(GENDIR)",
            "--prefix=" + name,
            "--config_file=$(location " + name + ".py)",
            "--source_file=$(location " + name + "_types.c)",
            "--header_file=$(location " + name + "_types.h)",
        ]),
    )

def ads7828_genrule(name, deps, **kwargs):
    _monitor_genrule("ads7828", name, deps, **kwargs)

def analog_genrule(name, deps, **kwargs):
    _monitor_genrule("analog", name, deps, **kwargs)

def bq34z100_genrule(name, deps, **kwargs):
    _monitor_genrule("bq34z100", name, deps, **kwargs)

def ina219_genrule(name, deps, **kwargs):
    _monitor_genrule("ina219", name, deps, **kwargs)

def ltc2309_genrule(name, deps, **kwargs):
    _monitor_genrule("ltc2309", name, deps, **kwargs)

def ltc4151_genrule(name, deps, **kwargs):
    _monitor_genrule("ltc4151", name, deps, **kwargs)

def ltc6804_genrule(name, deps, **kwargs):
    _monitor_genrule("ltc6804", name, deps, **kwargs)

def mcp342x_genrule(name, deps, **kwargs):
    _monitor_genrule("mcp342x", name, deps, **kwargs)

def mcp9800_genrule(name, deps, **kwargs):
    _monitor_genrule("mcp9800", name, deps, **kwargs)

def si7021_genrule(name, deps, **kwargs):
    _monitor_genrule("si7021", name, deps, **kwargs)
