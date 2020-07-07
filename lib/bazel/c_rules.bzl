"""This module contains rules for generating C libraries."""

# Most warnings are specified in lib/bazel/CROSSTOOL, but C-only warnings must
# be handled here. (CROSSTOOL can specify C++-only options, but not C-only.)
C_OPTS = [
    "-std=c99",
    "-Wmissing-prototypes",
    "-Wnested-externs",
    "-Wstrict-prototypes",
]

GCC_C_ONLY_WARNINGS = [
    "-Wjump-misses-init",
]

DEFAULT_CC_LINKOPTS = select({
    "//lib/bazel:q7_mode": ["-lstdc++"],
    "//conditions:default": [],
})

# Generates a "nostatic" version of a .c file, in which "static" is stripped
# from all functions.
def nostatic_c_file(c_file):
    output = "nostatic/" + c_file

    native.genrule(
        name = "gen_" + c_file[:-2] + "_nostatic",
        srcs = [c_file],
        outs = [output],
        cmd = " ".join([
            "PATH=/bin sed -E",
            "'s/^(static inline|static) ([^=]*\()/\\2/' $< > $@",
        ]),
    )

# Constructs the full set of copts from those specified for a particular rule.
def get_c_opts(copts_in):
    return select({
        "//lib/bazel:k8_gcc_mode": C_OPTS + GCC_C_ONLY_WARNINGS,
        "//lib/bazel:q7_mode": C_OPTS + GCC_C_ONLY_WARNINGS,
        "//conditions:default": C_OPTS,
    }) + copts_in

def makani_c_library(
        name,
        hdrs = [],
        srcs = [],
        deps = [],
        copts = [],
        linkopts = [],
        defines = [],
        undefined_symbols = [],
        nostatic_files = [],
        **kwargs):
    """Wrapper around cc_library.

    Also supports stripping of "static" from functions for use with tests. The
    argument `nostatic_files` may be passed with a list of files that are
    sources of this target. If nonempty, an additional target, with "_nostatic"
    appended to the name, is created in which "static" is stripped from all
    functions in the nostatic_files.

    Args:
        name: Name of the library.
        hdrs: See native cc_library rule.
        srcs: See native cc_library rule.
        deps: See native cc_library rule. If you want to use a selector, pass in
            the dict to which "select" should be applied.
        copts: See native cc_library rule.
        linkopts: See native cc_library rule.
        defines: See native cc_library rule.
        undefined_symbols: List of undefined symbols to provide to linker.
        nostatic_files: List of files in the srcs attribute If nonempty, an
            additional target, with "_nostatic" appended to the name, is created
            in which "static" is stripped from all functions in the
            nostatic_files.
        **kwargs: Additional arguments to pass to the native cc_library rule.
    """
    final_copts = get_c_opts(copts)

    linkopts2 = linkopts + ["-Wl,--undefined=" + s for s in undefined_symbols]

    # Setting linkstatic=1 tells Bazel not to consider generating a shared library
    # (.so file) from this rule. Our TMS570 build is not compatible with shared
    # libraries, and without this option, trying to directly build any
    # makani_c_library under the TMS570 configuration (i.e. "bazel build --config
    # tms570 <library_target_name>") would fail.
    native.cc_library(
        name = name,
        hdrs = hdrs,
        srcs = srcs,
        defines = defines,
        copts = final_copts,
        deps = deps,
        linkstatic = 1,
        linkopts = linkopts2,
        **kwargs
    )

    # Placing the nostatic files in a subdirectory rather than altering their
    # basenames ensures that SetState() will still work in unit tests.
    if nostatic_files:
        nostatic_srcs = []
        for s in srcs:
            if s in nostatic_files:
                nostatic_c_file(s)
                nostatic_srcs += ["nostatic/" + s]
            else:
                nostatic_srcs += [s]

        # Static functions typically don't have prototypes, so once they're no
        # longer static, they'll trigger a missing prototype warning if we let them.
        nostatic_c_opts = []
        for w in C_OPTS:
            if w != "-Wmissing-prototypes":
                nostatic_c_opts += [w]

        native.cc_library(
            name = name + "_nostatic",
            hdrs = hdrs,
            srcs = nostatic_srcs,
            deps = deps,
            copts = copts + nostatic_c_opts,
            defines = defines + ["MAKANI_TEST"],
            linkopts = linkopts2,
            **kwargs
        )

def makani_c_binary(name, linkshared = 0, copts = [], archs = [], tags = [], **kwargs):
    final_copts = get_c_opts(copts)
    native.cc_binary(
        name = name,
        copts = final_copts,
        linkshared = linkshared,
        tags = tags + ["arch:%s" % a for a in archs],
        **kwargs
    )

def makani_cc_library(linkopts = [], **kwargs):
    native.cc_library(
        linkopts = linkopts + DEFAULT_CC_LINKOPTS,
        **kwargs
    )

def makani_cc_binary(
        name,
        linkopts = [],
        linkshared = 0,
        archs = [],
        tags = [],
        **kwargs):
    native.cc_binary(
        name = name,
        linkopts = linkopts + DEFAULT_CC_LINKOPTS,
        linkshared = linkshared,
        tags = tags + ["arch:%s" % a for a in archs],
        **kwargs
    )

def makani_cc_test(
        name,
        srcs,
        deps = [],
        data = [],
        copts = [],
        linkopts = [],
        size = "small",
        args = [],
        **kwargs):
    deps2 = deps + ["@googletest//:gtest_lib"]
    native.cc_test(
        name = name,
        srcs = srcs,
        deps = deps2,
        copts = copts,
        linkopts = (linkopts + DEFAULT_CC_LINKOPTS),
        data = data,
        defines = ["MAKANI_TEST"],
        size = size,
        args = args,
        **kwargs
    )
