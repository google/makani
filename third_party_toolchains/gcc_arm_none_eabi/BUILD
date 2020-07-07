licenses(["restricted"])  # GPLv3, portions LGPLv3, BSD-like, GCC Runtime Library Exception

exports_files(["LICENSE"])

package(default_visibility = ["//visibility:public"])

# All files needed by the toolchain, for convenient reference.
filegroup(
    name = "everything",
    srcs = glob([
        "arm-none-eabi/**",
        "bin/**",
        "lib/**",
        "libexec/**",
        "share/**",
    ]),
)
