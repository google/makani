"""This module contains swig rules."""

load("//lib/bazel:c_rules.bzl", "makani_c_binary", "makani_cc_binary")
load("//lib/bazel:py_rules.bzl", "py_library")
load("//lib/bazel:rule_util.bzl", "gather_headers_and_include_dirs")

def _swig_wrap_py_impl(ctx):
    swig_interfaces = ctx.files.swig_interfaces
    generated_wrap_src = ctx.outputs.generated_wrap_src
    generated_swig_py = ctx.outputs.generated_swig_py
    main_interface = swig_interfaces[0]

    header_files, include_dirs = gather_headers_and_include_dirs(
        ctx.attr.header_providers,
    )

    # Add `genfiles` to the include directories.
    defines_str = " ".join(["-D" + define for define in ctx.attr.defines])
    includes_str = " ".join(["-I" + d for d in include_dirs])
    swig_command = "swig3.0 %s -module %s -python %s %s -o %s %s" % (
        "-c++" if ctx.attr.is_cxx else "",
        ctx.attr.module_name,
        defines_str,
        includes_str,
        generated_wrap_src.path,
        main_interface.path,
    )

    # TODO: Try to convert this rule to run.
    ctx.actions.run_shell(
        inputs = swig_interfaces + header_files,
        outputs = [generated_wrap_src, generated_swig_py],
        command = swig_command,
        mnemonic = "Generating",
    )

    # buildifier: disable=rule-impl-return
    return struct(transitive_files = [generated_swig_py])

swig_py_interface_cc = rule(
    implementation = _swig_wrap_py_impl,
    attrs = {
        "defines": attr.string_list(mandatory = False),
        "header_providers": attr.label_list(allow_files = True),
        "is_cxx": attr.bool(default = True),
        "module_name": attr.string(),
        "swig_interfaces": attr.label_list(
            allow_files = [".i", ".swig"],
            mandatory = True,
        ),
    },
    outputs = {
        "generated_swig_py": "%{module_name}.py",
        "generated_wrap_src": "%{name}_wrap.cc",
    },
)

swig_py_interface_c = rule(
    implementation = _swig_wrap_py_impl,
    attrs = {
        "defines": attr.string_list(mandatory = False),
        "header_providers": attr.label_list(allow_files = True),
        "is_cxx": attr.bool(default = False),
        "module_name": attr.string(),
        "swig_interfaces": attr.label_list(
            allow_files = [".i", ".swig"],
            mandatory = True,
        ),
    },
    outputs = {
        "generated_swig_py": "%{module_name}.py",
        "generated_wrap_src": "%{name}_wrap.c",
    },
)

def _swig_shared_library(name, type, copts = [], deps = [], **kwargs):
    copts2 = copts + [
        "-fpic",
        "-Wno-cast-qual",
        "-Wno-shadow",
        "-Wno-conversion",
    ]
    deps2 = deps + ["@usr_include//:python_includes"]
    if type == "c":
        copts2 += [
            "-Wno-missing-prototypes",
            "-Wno-strict-prototypes",
            "-Wno-stack-protector",
        ]
        makani_c_binary(
            name = name,
            copts = copts2,
            linkshared = 1,
            deps = deps2,
            **kwargs
        )
    elif type == "cc":
        copts2 += ["-Wno-effc++", "-Wno-deprecated-register"]
        makani_cc_binary(
            name = name,
            copts = copts2,
            linkshared = 1,
            deps = deps2,
            **kwargs
        )
    else:
        fail("'type' attribute must be 'c' or 'cc'.")

def _swig_rule_names(swig_module_name, type):
    # The name of the lib must be the name of the output shared library.
    wrap_lib_name = "_" + swig_module_name + ".so"

    wrap_creator_name = swig_module_name + "_creator"

    if type == "c":
        wrap_file = wrap_creator_name + "_wrap.c"
    elif type == "cc":
        wrap_file = wrap_creator_name + "_wrap.cc"
    else:
        fail("'type' attribute must be 'c' or 'cc'.")

    return wrap_creator_name, wrap_lib_name, wrap_file

def _py_wrap_template(
        name,
        type,
        swig_interfaces,
        module_name = None,
        srcs = [],
        c_deps = [],
        py_deps = [],
        data = [],
        defines = [],
        **kwargs):
    """The template to generate SWIG rule for C/C++ modules.

    Args:
      name: Name of the rule.
      type: 'c' or 'cc'.
      swig_interfaces: List of swig interface files. The first should be the
          main interface which includes the rest.
      module_name: Name of the SWIG module. By default it is the basename for
          the first SWIG interface file.
      srcs: List of source files.
      c_deps: List of C dependencies for the swig shared library.
      py_deps: List of Python dependencies for the swig shared library.
      data: List of data items for the py_library.
      defines: List passed to the swig py interface c/cc rules.
      **kwargs: Additional arguments passed to the swig shared library rule.
    """
    main_interface = swig_interfaces[0]

    if module_name == None:
        module_name = main_interface[:main_interface.rfind(".")]

    wrap_creator_name, wrap_lib_name, wrap_file = _swig_rule_names(
        module_name,
        type,
    )

    src_headers = [f for f in srcs if f.endswith(".h")]

    if type == "c":
        swig_py_interface_c(
            name = wrap_creator_name,
            module_name = module_name,
            swig_interfaces = swig_interfaces,
            header_providers = src_headers + c_deps,
            defines = defines,
        )
    elif type == "cc":
        swig_py_interface_cc(
            name = wrap_creator_name,
            module_name = module_name,
            swig_interfaces = swig_interfaces,
            header_providers = src_headers + c_deps,
            defines = defines,
        )
    else:
        fail("'type' attribute must be 'c' or 'cc'.")

    _swig_shared_library(
        name = wrap_lib_name,
        type = type,
        srcs = srcs + [wrap_file],
        deps = c_deps,
        defines = defines,
        **kwargs
    )

    # Concatenation of wrap_creator and wrap_lib rules.
    py_library(
        name = name,
        srcs = [module_name + ".py"],
        deps = py_deps,
        data = data + [wrap_lib_name],
    )

def py_wrap_c(name, **kwargs):
    _py_wrap_template(name, "c", **kwargs)

def py_wrap_cc(name, **kwargs):
    _py_wrap_template(name, "cc", **kwargs)
