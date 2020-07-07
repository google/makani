"""This module contains autogen bazel rules."""

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:c_rules.bzl", "makani_c_binary", "makani_c_library")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library")
load("//lib/bazel:rule_util.bzl", "gather_headers_and_include_dirs")

def check_for_make_artifacts(autogen_files):
    existing_files = native.glob(autogen_files)
    if existing_files:
        fail("\n".join(
            ["The following files are already in the source tree:"] +
            existing_files +
            ["You probably need to run 'make clean_all'. (This error " +
             "prevents Bazel from using artifacts of a Make build.)"],
        ))

def _run_h2py_impl(ctx):
    script = ctx.executable.script
    header_files, include_dirs = gather_headers_and_include_dirs(
        ctx.attr.header,
        ctx.attr.header_providers,
    )

    args = [
        "--header",
        ctx.file.header.path,
        "--output",
        ctx.outputs.h2py_module.path,
    ]

    for d in include_dirs:
        args += ["--include_dir", d]

    so_files = [f for f in ctx.files.shared_libs if f.path.endswith(".so")]
    if so_files:
        args += [
            "--shared_libs",
            ",".join([s.path for s in so_files]),
            "--shared_lib_root",
            ctx.configuration.bin_dir.path,
        ]
    if ctx.attr.defines:
        args += ["--defines", ",".join([d for d in ctx.attr.defines])]

    ctx.actions.run(
        inputs = ctx.files.shared_libs + header_files,
        outputs = [ctx.outputs.h2py_module],
        executable = script,
        arguments = args,
        mnemonic = "Generating",
    )

_run_h2py = rule(
    implementation = _run_h2py_impl,
    attrs = {
        "defines": attr.string_list(),
        "h2py_module": attr.output(mandatory = True),
        "header": attr.label(allow_single_file = [".h"], mandatory = True),
        "header_providers": attr.label_list(allow_files = True),
        "script": attr.label(executable = True, mandatory = True, cfg = "host"),
        "shared_libs": attr.label_list(allow_files = False),
    },
)

def h2py_library(
        name,
        header,
        header_deps = [],
        function_deps = [],
        **library_kwargs):
    """Builds an h2py-generated Python library from C sources.

    Args:
        name: Name of the rule.
        header: Name of the header file to which h2py is applied.
        header_deps: C libraries that transitively expose header dependencies.
        function_deps: Dependencies with functions that will be exposed by the
            h2py_library (and thus must be compiled into a shared library), in
            addition to exposing transitive header dependencies.
        **library_kwargs: Any generic kwargs (e.g. visibility) that should be
            passed to the generated library.

    All header dependencies must be either transitively included in
    `header_deps` or `function_deps`, or the library will not build
    successfully.
    """
    if not name.startswith("h2py_"):
        fail("Target %s: The name of an h2py_library must begin with 'h2py_'." %
             name)

    output_file = name[5:] + ".py"
    check_for_make_artifacts([output_file])

    shared_libs = []
    if function_deps:
        shared_lib_name = "_" + name[5:] + ".so"
        makani_c_binary(
            name = shared_lib_name,
            deps = function_deps,
            copts = ["-fpic"],
            linkshared = 1,
        )
        shared_libs += [shared_lib_name]

    _run_h2py(
        name = name + "-run_h2py",
        script = "//lib/python/autogen:h2py",
        header = header,
        header_providers = header_deps + function_deps,
        h2py_module = output_file,
        shared_libs = shared_libs,
    )

    py_library(
        name = name,
        srcs = [output_file],
        data = shared_libs,
        srcs_version = "PY2AND3",
        **library_kwargs
    )

def _run_pack_impl(ctx):
    script = ctx.executable.script
    header_files, include_dirs = gather_headers_and_include_dirs(
        ctx.attr.header,
        ctx.attr.header_providers,
    )

    args = [
        "-e",
        "big",
        "--output_prefix",
        ctx.outputs.pack_header.path[:-2],
        "--autogen_root",
        ctx.configuration.genfiles_dir.path,
    ]
    for d in include_dirs:
        args += ["--include_dir", d]
    args += [ctx.file.header.path]

    ctx.actions.run(
        inputs = list(header_files),
        outputs = [ctx.outputs.pack_header, ctx.outputs.pack_source],
        executable = script,
        arguments = args,
        mnemonic = "Generating",
    )

_run_pack = rule(
    implementation = _run_pack_impl,
    attrs = {
        "header": attr.label(allow_single_file = [".h"], mandatory = True),
        "header_providers": attr.label_list(allow_files = True),
        "pack_header": attr.output(mandatory = True),
        "pack_source": attr.output(mandatory = True),
        "script": attr.label(executable = True, mandatory = True, cfg = "host"),
    },
    output_to_genfiles = True,
)

def pack_library(name, header, deps = [], **library_kwargs):
    """Generates a libary of pack functions for the supplied header file.

    Args:
        name: Name of the rule.
        header: Name of the input header file.
        deps: C library dependencies. These must transitively expose (via
            `hdrs`) all required header files.
        **library_kwargs: Any generic kwargs (e.g. visibility) that should be
            passed to the generated library.
    """
    if not name.startswith("pack_"):
        fail("Target %s: The name of a pack_library target must begin with 'pack_'." %
             name)

    script_name = name + "-build_script"
    pack_header = name + ".h"
    pack_source = name + ".c"
    check_for_make_artifacts([pack_header, pack_source])

    _run_pack(
        name = name + "-run_pack",
        script = "//lib/pack:pack",
        header = header,
        header_providers = deps,
        pack_header = pack_header,
        pack_source = pack_source,
    )

    makani_c_library(
        name = name,
        hdrs = [pack_header, header],
        srcs = [pack_source],
        deps = deps,
        copts = ["-Wno-unused-function"],
        **library_kwargs
    )

def _run_generate_cvt_lib_impl(ctx):
    pack_pylib = ctx.attr.pack_pylib
    name = pack_pylib.label.name
    if name.startswith("h2py_"):
        filename = name[5:] + ".py"
    else:
        fail("pack_pylib must start with 'h2py_'.")

    script = ctx.executable.script
    runfiles_path = script.path + ".runfiles"
    pack_pylib_path = pack_pylib.label.package + "/" + filename

    ctx.actions.run(
        inputs = pack_pylib[PyInfo].transitive_sources.to_list(),
        outputs = ctx.outputs.outputs,
        executable = script,
        arguments = [
            "--h2py_dir",
            runfiles_path,
            "--autogen_root",
            ctx.configuration.genfiles_dir.path,
            runfiles_path + "/" + pack_pylib_path,
        ],
        mnemonic = "Generating",
    )

_run_generate_cvt_lib = rule(
    implementation = _run_generate_cvt_lib_impl,
    attrs = {
        "outputs": attr.output_list(mandatory = True),
        "pack_pylib": attr.label(allow_files = False, cfg = "host"),
        "script": attr.label(allow_files = False, executable = True, cfg = "host"),
    },
    output_to_genfiles = True,
)

def cvt_library(name, pack_pylib, expose_hdrs, deps, **library_kwargs):
    """Generates a makani_c_library of CVT accessors.

    Args:
        name: Name of the library.
        pack_pylib: Dependencies for the py_binary.
        expose_hdrs: Headers passed to the makani_c_library.
        deps: Dependencies for the makani_c_library.
        **library_kwargs: Any generic kwargs (e.g. visibility) that should be
            passed to the generated library.

    This rule has to build its own binary of the CVT code-generation script to
    allow its h2py pack module to be imported.
    """
    script_name = name + "-build_script"
    script_target = ":" + script_name
    cvt_header = name + ".h"
    cvt_source = name + ".c"

    check_for_make_artifacts([cvt_header, cvt_source])

    py_binary(
        name = script_name,
        main = "//avionics/common:generate_cvt_lib.py",
        srcs = ["//avionics/common:generate_cvt_lib.py"],
        deps = [
            pack_pylib,
            "//avionics/network:h2py_message_type",
            "//lib/python:c_helpers",
            requirement("python-gflags"),
        ],
    )

    _run_generate_cvt_lib(
        name = name + "-run_build_script",
        script = script_target,
        pack_pylib = pack_pylib,
        outputs = [cvt_header, cvt_source],
    )

    makani_c_library(
        name = name,
        hdrs = [cvt_header] + expose_hdrs,
        srcs = [cvt_source],
        deps = deps + ["//avionics/common:cvt"],
        **library_kwargs
    )

def message_ruleset(
        name,
        header,
        source = None,
        deps = None,
        generate_h2py_lib = True,
        generate_cvt_lib = True,
        **library_kwargs):
    """Generates a collection of rules from a C header of message types.

    Args:
        name: Base rule name.
        header: C header file.
        source: Optional C source file, implementing the header as necessary.
        deps: C library dependencies.
        generate_h2py_lib: Whether to generate an h2py_library.
        generate_cvt_lib: Whether to generate a library of CVT accessors.
            If True, generate_h2py_lib must be true as well.
        **library_kwargs: Any generic kwargs (e.g. visibility) that should be
            passed to the generated libraries.

    Generated rules:
        - <name>: A makani_c_library consisting of only `header` and `source`.
        - pack_<name>: A library of pack functions for the messeages in
                `header`.
        - h2py_pack_<name>: An h2py library including both the message
                definitions and pack functions.
        - cvt_<name>: A C library of CVT accessors.
    """
    if generate_cvt_lib and not generate_h2py_lib:
        fail("If generate_cvt_lib is True, then generate_h2py_lib must be " +
             "True as well.")
    if source:
        srcs = [source]
    else:
        srcs = []

    if not deps:
        deps = []

    pack_name = "pack_" + name
    h2py_pack_name = "h2py_" + pack_name
    cvt_name = "cvt_" + name

    makani_c_library(
        name = name,
        hdrs = [header],
        srcs = srcs,
        deps = deps,
        **library_kwargs
    )

    pack_library(
        name = pack_name,
        header = header,
        deps = [":" + name],
        **library_kwargs
    )

    if generate_h2py_lib:
        h2py_library(
            name = h2py_pack_name,
            header = pack_name + ".h",
            function_deps = [pack_name],
            **library_kwargs
        )

    if generate_h2py_lib and generate_cvt_lib:
        cvt_library(
            name = cvt_name,
            pack_pylib = ":" + h2py_pack_name,
            expose_hdrs = [header, pack_name + ".h"],
            deps = deps + [":" + name, ":" + pack_name],
            **library_kwargs
        )
