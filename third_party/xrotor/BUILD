package(default_visibility = ["//visibility:public"])
licenses(["notice"])  # GPL

exports_files(["plotlib/Xwin.c"])

load("//lib/bazel:drela_rules.bzl", "drela_archive", "drela_fortran_binary")

drela_archive(
    name = "xwin",
    srcs = ["plotlib/Xwin.c"],
    defines = ["UNDERSCORE", "NDEBUG"],
    copts = ["-O2"],
)

drela_archive(
    name = "plotlib",
    srcs = [
        "plotlib/gw_subs.f",
        "plotlib/plt_3D.f",
        "plotlib/plt_base.f",
        "plotlib/plt_color.f",
        "plotlib/plt_font.f",
        "plotlib/plt_old.f",
        "plotlib/plt_util.f",
        "plotlib/ps_subs.f",
        "plotlib/ps_subs_old.f",
        "plotlib/set_subs.f",
        "plotlib/util-ops.f",
    ],
    includes = [
        "plotlib/CHAR.INC",
        "plotlib/MATH.INC",
        "plotlib/SLAN.INC",
        "plotlib/SYMB.INC",
        "plotlib/colors.inc",
        "plotlib/masks.inc",
        "plotlib/pltlib.inc",
        "plotlib/version.inc",
    ],
    defines = ["UNDERSCORE"],
    copts = [
        "-O2",
        "-fdefault-real-8",
    ],
)

drela_fortran_binary(
    name = "jplot",
    srcs = [
        "src/jplot.f",
        "src/jputil.f",
        "src/userio.f",
        "src/xutils.f",
    ],
    # Do not sort: "libxwin.a" must come after "libplotlib.a".
    archives = [
        "libplotlib.a",
        "libxwin.a",
    ],
    copts = [
        "-O2",
        "-w",
        "-fdefault-real-8",
    ],
)

drela_fortran_binary(
    name = "jplote",
    srcs = [
        "src/jplote.f",
        "src/jputil.f",
        "src/userio.f",
        "src/xutils.f",
    ],
    # Do not sort: "libxwin.a" must come after "libplotlib.a".
    archives = [
        "libplotlib.a",
        "libxwin.a",
    ],
    copts = [
        "-O2",
        "-fdefault-real-8",
    ],
)

drela_fortran_binary(
    name = "xrotor",
    srcs = [
        "src/jputil.f",
        "src/modify.f",
        "src/plotdata.f",
        "src/plutil.f",
        "src/spline.f",
        "src/srclin.f",
        "src/userio.f",
        "src/vortex.f",
        "src/xaero.f",
        "src/xbend.f",
        "src/xcasepl.f",
        "src/xdesi.f",
        "src/xinte.f",
        "src/xio.f",
        "src/xjmap.f",
        "src/xmodi.f",
        "src/xnoise.f",
        "src/xoper.f",
        "src/xrotor.f",
        "src/xrotpl.f",
        "src/xutils.f",
    ],
    # Do not sort: "libxwin.a" must come after "libplotlib.a".
    archives = [
        "libplotlib.a",
        "libxwin.a",
    ],
    includes = [
        "src/XROTOR.INC",
    ],
    copts = [
        "-O2",
        "-fdefault-real-8",
    ],
)
