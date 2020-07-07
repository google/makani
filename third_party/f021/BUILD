# Bazel doesn't like the ".lib" suffix in cc_library srcs.
genrule(
    name = "copy_lib",
    srcs = ["F021_API_CortexR4_BE_V3D16.lib"],
    outs = ["F021_API_CortexR4_BE_V3D16.a"],
    cmd = "cp $< $@",
)

cc_library(
    name = "f021",
    hdrs = glob(["include/*.h"]),
    srcs = ["F021_API_CortexR4_BE_V3D16.a"],
    includes = ["include"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "f021_transitive_headers",
    hdrs = [],
)
