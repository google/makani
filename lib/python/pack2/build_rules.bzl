"""This module contains build rules for pack2."""

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:c_rules.bzl", "makani_c_library")
load("//lib/bazel:py_rules.bzl", "py_library")

def pack2(name, src, deps = []):
    """Builds a pack2 library.

    Args:
        name: Name of the library.
        src: Source file.
        deps: List of dependencies.
    """
    native.genrule(
        name = name + "_genrule",
        srcs = [src] + deps,
        outs = [
            name + ".c",
            name + ".h",
            name + ".py",
        ],
        tools = ["//lib/python/pack2/tools:p2generate"],
        cmd = " ".join([
            "$(location //lib/python/pack2/tools:p2generate)",
            "--output_c_header=$(location " + name + ".h)",
            "--output_c_source=$(location " + name + ".c)",
            "--output_py=$(location " + name + ".py)",
            "--base_dir=$(GENDIR)",
            "--input=$(location " + src + ")",
        ]),
    )

    native.filegroup(
        name = name,
        srcs = [src] + deps,
    )

    deps_c = [dep + "_c" for dep in deps]

    makani_c_library(
        name = name + "_c",
        hdrs = [name + ".h"],
        srcs = [name + ".c"],
        defines = select({
            "//lib/bazel:tms570_mode": ["PACK2_FLASH_POINTERS"],
            "//conditions:default": [],
        }),
        deps = select({
            "//lib/bazel:tms570_mode": deps_c + [
                "//avionics/firmware/startup:ldscript",
            ],
            "//conditions:default": deps_c,
        }),
        visibility = ["//visibility:public"],
    )

    deps_py = [dep + "_py" for dep in deps]

    py_library(
        name = name + "_py",
        srcs = [name + ".py"],
        deps = deps_py + [
            "//lib/python/pack2:pack2",
            "//lib/python/pack2:py_types",
            requirement("PyYAML"),
        ],
        visibility = ["//visibility:public"],
    )

def _internal_packdb_impl(ctx):
    py = ""
    for m in ctx.attr.modules:
        py += "import " + m + "\n"
    output = ctx.outputs.out
    ctx.actions.write(output = output, content = py)

_internal_packdb = rule(
    implementation = _internal_packdb_impl,
    attrs = {
        "out": attr.string(),
        "modules": attr.string_list(),
    },
    outputs = {"out": "%{out}.py"},
)

def packdb(name, packs, visibility):
    """Pack database rule.

    Args:
        name: Name of the library.
        packs: List of packs to include.
        visibility: Sets the visibility in the py_library rule.
    """
    modules = [
        p.replace("//", "/").replace("/", ".").replace(":", ".")
        for p in packs
    ]
    modules = ["makani" + m for m in modules]

    _internal_packdb(name = name + "_py", out = name, modules = modules)
    lib_deps = [p + "_py" for p in packs]
    py_library(
        name = name,
        srcs = [name + "_py"],
        deps = lib_deps,
        visibility = visibility,
    )
