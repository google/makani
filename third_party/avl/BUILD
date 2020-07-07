package(default_visibility = ["//visibility:public"])
licenses(["notice"])  # GPL

exports_files(["plotlib/Xwin2.c"])

load("//lib/bazel:drela_rules.bzl", "drela_archive", "drela_fortran_binary")

drela_archive(
    name = "xwin",
    srcs = ["plotlib/Xwin2.c"],
    defines = ["UNDERSCORE", "NDEBUG"],
    copts = ["-O2"],
)

drela_archive(
    name = "plotlib",
    srcs = [
        "plotlib/gw_subs.f",
        "plotlib/plt_base.f",
        "plotlib/plt_color.f",
        "plotlib/plt_font.f",
        "plotlib/plt_util.f",
        "plotlib/ps_subs.f",
        "plotlib/set_subs.f",
    ],
    includes = [
        "plotlib/CHAR.INC",
        "plotlib/MATH.INC",
        "plotlib/SLAN.INC",
        "plotlib/SYMB.INC",
        "plotlib/pltlib.inc",
    ],
    defines = ["UNDERSCORE"],
    copts = [
        "-O2",
        "-fdefault-real-8",
    ],
)

drela_archive(
    name = "eispack",
    srcs = ["eispack/eispack.f"],
    copts = [
        "-O2",
        "-fdefault-real-8",
        "-fdefault-double-8",
    ],
)

drela_fortran_binary(
    name = "avl",
    srcs = [
        "src/avl.f",
        "src/aoper.f",
        "src/amode.f",
        "src/amass.f",
        "src/asetup.f",
        "src/amake.f",
        "src/ainput.f",
        "src/aoutput.f",
        "src/aero.f",
        "src/atrim.f",
        "src/atpforc.f",
        "src/aic.f",
        "src/cdcl.f",
        "src/airutil.f",
        "src/autil.f",
        "src/aplotvl.f",
        "src/aplottp.f",
        "src/aplotmd.f",
        "src/limits.f",
        "src/pltops.f",
        "src/hidden.f",
        "src/plsubs.f",
        "src/userio.f",
        "src/plutil.f",
        "src/arrow3d.f",
        "src/getvm.f",
        "src/matrix.f",
        "src/spline.f",
        "src/sgutil.f",
        "src/second_g77.f",
    ],
    # Do not sort: "libxwin.a" must come after "libplotlib.a".
    archives = [
        "libplotlib.a",
        "libeispack.a",
        "libxwin.a",
    ],
    includes = [
        "src/AINDEX.INC",
        "src/AVL.INC",
        "src/AVLPLT.INC",
        "src/MASKS.INC",
    ],
    copts = [
        "-O2",
        "-fdefault-real-8",
    ],
)
