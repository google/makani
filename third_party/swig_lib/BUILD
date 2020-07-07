package(default_visibility = ["//visibility:public"])

# These files should be updated to match the numpy version in use.  They cannot
# be loaded directly with git_repository since we need generated header files
# from the build, so they are taken from the installed python package.
# Currently we are using numpy 1.14.3.

exports_files(["numpy.i"])

cc_library(
    name = "numpy_headers",
    hdrs = glob(["include/numpy/*.h"]),
    includes = ["include"],
)
