"""This module contains build rules for tms570."""

load("//lib/bazel:c_rules.bzl", "makani_c_binary")

def _get_linkopts(name):
    return (["-Wl,-Map=$(BINDIR)/%s/%s.map" % (native.package_name(), name)])

def _tms570_binary(name, **kwargs):
    makani_c_binary(
        name = name,
        archs = ["tms570"],
        **kwargs
    )

def tms570_application(name, srcs, deps, **kwargs):
    _tms570_binary(
        name = name,
        srcs = srcs,
        deps = deps + [
            "//avionics/firmware/startup:application",
            "//avionics/firmware/util:assert",
            "//avionics/firmware/util:newlib",
        ],
        linkopts = _get_linkopts(name),
        **kwargs
    )

def tms570_bootloader(name, deps = [], **kwargs):
    _tms570_binary(
        name = name,
        deps = deps + [
            "//avionics/firmware/cpu:on_fatal_stub",
            "//avionics/firmware/identity:identity",
            "//avionics/firmware/startup:bootloader",
            "//avionics/firmware/util:assert",
            "//avionics/firmware/util:newlib",
            "//avionics/bootloader/firmware:bootloader_main",
        ],
        linkopts = _get_linkopts(name),
        **kwargs
    )

def tms570_bootloader_application(name, deps = [], **kwargs):
    _tms570_binary(
        name = name,
        deps = deps + [
            "//avionics/firmware/cpu:on_fatal_stub",
            "//avionics/firmware/identity:identity",
            "//avionics/firmware/startup:application_ram",
            "//avionics/firmware/util:assert",
            "//avionics/firmware/util:newlib",
            "//avionics/bootloader/firmware:bootloader_application_main",
        ],
        linkopts = _get_linkopts(name),
        **kwargs
    )

def tms570_test(name, srcs, deps, **kwargs):
    _tms570_binary(
        name = name,
        srcs = srcs,
        deps = deps + [
            "//avionics/firmware/startup:application",
            "//avionics/firmware/test:test",
            "//avionics/firmware/util:assert",
            "//avionics/firmware/util:newlib",
        ],
        tags = ["tms570_test"],
        linkopts = _get_linkopts(name),
        **kwargs
    )

def configurable_macro(name, low, high, default):
    """Sets up a numeric macro specified at the command line using --define.

    Only values from `low` to `high` inclusive can be specified. (This is a
    limitation of the config_setting system, not by design.) If no value or an
    unrecognized value is specified, then the default will be used.

    For example,
        configurable_macro("FOO", 1, 100, 50)
        returns a select() that will define "FOO=50" by default, and which will
        recognize "--define FOO=<N>" for any N in range(1, 101).

    Args:
        name: Name of the macro.
        low: Lower bound for the macro value.
        high: Upper bound for the macro value.
        default: The default value for the macro.

    Returns:
        A selector to be passed to cc_library or cc_binary's `define` attribute.

    Note: If we ever want to call this more than once for a given macro, this
    implementation needs to be changed.
    """
    selector = {}
    for i in range(low, high + 1):
        setting = "%s=%d" % (name, i)
        native.config_setting(
            name = setting,
            values = {"define": setting},
        )
        selector[":" + setting] = [setting]
        if i == default:
            selector["//conditions:default"] = [setting]
    return select(selector)
