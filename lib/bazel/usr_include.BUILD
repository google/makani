load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gflags",
    hdrs = glob(["gflags/**/*.h"]),
    deps = ["@usr_lib//:gflags"],
)

cc_library(
    name = "glib",
    hdrs = glob(["glib-2.0/**/*.h"]),
    includes = ["glib-2.0"],
    deps = ["@usr_lib//:glib"],
)

cc_library(
    name = "glog",
    hdrs = glob(["glog/**/*.h"]),
    deps = ["@usr_lib//:glog"],
)

cc_library(
    name = "gsl",
    hdrs = glob(["gsl/**/*.h"]),
    deps = ["@usr_lib//:gsl"],
)

cc_library(
    name = "gtk",
    hdrs = glob([
        "atk-1.0/**/*.h",
        "cairo/**/*.h",
        "gdk-pixbuf-2.0/**/*.h",
        "gtk-2.0/**/*.h",
        "gtkextra-3.0/**/*.h",
        "pango-1.0/**/*.h",
    ]),
    includes = [
        "atk-1.0",
        "cairo",
        "gdk-pixbuf-2.0",
        "gtk-2.0",
        "gtkextra-3.0",
        "pango-1.0",
    ],
    deps = [
        ":glib",
        "@usr_lib//:gtk",
    ],
)

cc_library(
    name = "hdf5",
    hdrs = glob([
        "hdf5/serial/*.h",
    ]),
    includes = ["hdf5/serial"],
)

cc_library(
    name = "jansson",
    hdrs = glob(["jansson/**/*.h"]),
    deps = ["@usr_lib//:jansson"],
)

cc_library(
    name = "pcap",
    hdrs = [
        "pcap.h",
        "pcap/pcap.h",
    ],
    deps = ["@usr_lib//:pcap"],
)

cc_library(
    name = "python_includes",
    hdrs = glob(["python2.7/**/*.h"]),
    includes = ["python2.7"],
)
