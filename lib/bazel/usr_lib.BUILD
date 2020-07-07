load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gflags",
    srcs = glob(["x86_64-linux-gnu/libgflags.so*"]),
    linkstatic = 1,
    deps = [":unwind"],
)

cc_library(
    name = "gflags_static",
    srcs = ["x86_64-linux-gnu/libgflags.a"],
    linkstatic = 1,
    deps = [":unwind_static"],
)

cc_library(
    name = "glib",
    srcs = glob(["x86_64-linux-gnu/libglib-2.0.so*"]),
    hdrs = glob([
        "x86_64-linux-gnu/glib-2.0/include/**/*.h",
    ]),
    includes = [
        "x86_64-linux-gnu/glib-2.0/include",
    ],
)

cc_library(
    name = "glog",
    srcs = glob(["x86_64-linux-gnu/libglog.so*"]),
    linkstatic = 1,
    deps = [":unwind"],
)

cc_library(
    name = "gsl",
    srcs = glob([
        "x86_64-linux-gnu/libgsl.so*",
        "x86_64-linux-gnu/libgslcblas.so*",
    ]),
    linkstatic = 1,
)

cc_library(
    name = "gtk",
    srcs = ["x86_64-linux-gnu/libpangoft2-1.0.so"] + glob([
        "x86_64-linux-gnu/libatk-1.0.so*",
        "x86_64-linux-gnu/libcairo.so*",
        "x86_64-linux-gnu/libfontconfig.so*",
        "x86_64-linux-gnu/libfreetype.so*",
        "x86_64-linux-gnu/libgdk-x11-2.0.so*",
        "x86_64-linux-gnu/libgdk_pixbuf-2.0.so*",
        "x86_64-linux-gnu/libgtkextra*.so*",
        "x86_64-linux-gnu/libgio-2.0.so*",
        "x86_64-linux-gnu/libgobject-2.0.so*",
        "x86_64-linux-gnu/libgtk-x11-2.0.so*",
        "x86_64-linux-gnu/libpango-1.0.so*",
        "x86_64-linux-gnu/libpangocairo-1.0.so*",
    ]),
    hdrs = glob(["x86_64-linux-gnu/gtk-2.0/include/**/*.h"]),
    includes = ["x86_64-linux-gnu/gtk-2.0/include"],
    # NOTE: Without this, Bazel warns: "setting 'linkstatic=1' is
    # recommended if there are no object files." I think this option only
    # concerns linking against libraries compiled from the rule, not those
    # directly provided via srcs. In any case, binaries do properly link
    # against the .so files with linkstatic=1.
    linkstatic = 1,
)

cc_library(
    name = "hdf5",
    srcs = glob(["x86_64-linux-gnu/libhdf5*.so*"]),
    linkstatic = 1,
    deps = ["@usr_include//:hdf5"],
)

cc_library(
    name = "jansson",
    srcs = glob(["x86_64-linux-gnu/libjansson.so*"]),
    linkstatic = 1,
)

cc_library(
    name = "pcap",
    srcs = glob(["x86_64-linux-gnu/libpcap*.so*"]),
    linkstatic = 1,
)

cc_library(
    name = "snappy",
    srcs = glob([
        "libsnappy*.so*",
        "x86_64-linux-gnu/libsnappy*.so*",
    ]),
    linkstatic = 1,
)

cc_library(
    name = "unwind",
    srcs = glob(["x86_64-linux-gnu/libunwind.so*"]),
    linkstatic = 1,
)

cc_library(
    name = "unwind_static",
    # The current Goobuntu version of libunwind uses liblzma but fails to link
    # against it explicitly.
    srcs = [
        "x86_64-linux-gnu/liblzma.a",
        "x86_64-linux-gnu/libunwind.a",
    ],
    linkstatic = 1,
)
