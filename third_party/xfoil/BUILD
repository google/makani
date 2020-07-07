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
    name = "os",
    srcs = [
        "osrc/getosfile.c",
        "osrc/osmap.f",
        "src/frplot.f",
        "src/ntcalc.f",
    ],
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
    name = "xfoil",
    srcs = [
        "src/aread.f",
        "src/blplot.f",
        "src/dplot.f",
        "src/gui.f",
        "src/iopol.f",
        "src/modify.f",
        "src/naca.f",
        "src/plutil.f",
        "src/pntops.f",
        "src/polplt.f",
        "src/profil.f",
        "src/sort.f",
        "src/spline.f",
        "src/userio.f",
        "src/xbl.f",
        "src/xblsys.f",
        "src/xfoil.f",
        "src/xgdes.f",
        "src/xgeom.f",
        "src/xmdes.f",
        "src/xoper.f",
        "src/xpanel.f",
        "src/xplots.f",
        "src/xpol.f",
        "src/xqdes.f",
        "src/xsolve.f",
        "src/xtcam.f",
        "src/xutils.f",
    ],
    # Do not sort: "libxwin.a" must come after "libplotlib.a".
    archives = [
        "libplotlib.a",
        "libxwin.a",
        "libos.a",
    ],
    includes = [
        "src/BLPAR.INC",
        "src/CIRCLE.INC",
        "src/PINDEX.INC",
        "src/XBL.INC",
        "src/XDES.INC",
        "src/XFOIL.INC",
    ],
    copts = [
        "-O2",
        "-w",
        "-fdefault-real-8",
    ],
)

drela_fortran_binary(
    name = "pplot",
    srcs = [
        "src/iopol.f",
        "src/polplt.f",
        "src/pplot.f",
        "src/sort.f",
        "src/userio.f",
    ],
    # Do not sort: "libxwin.a" must come after "libplotlib.a".
    archives = [
        "libplotlib.a",
        "libxwin.a",
    ],
    includes = [
        "src/PPLOT.INC",
        "src/PINDEX.INC",
    ],
    copts = [
        "-O2",
        "-w",
        "-fdefault-real-8",
    ],
)

drela_fortran_binary(
    name = "pxplot",
    srcs = [
        "src/gui.f",
        "src/plutil.f",
        "src/pxplot.f",
        "src/userio.f",
    ],
    # Do not sort: "libxwin.a" must come after "libplotlib.a".
    archives = [
        "libplotlib.a",
        "libxwin.a",
    ],
    includes = [
        "src/PXPLOT.INC",
    ],
    copts = [
        "-O2",
        "-fdefault-real-8",
    ],
)
