package(default_visibility=["//visibility:public"])

cc_library(
    name = "modbus",
    srcs = glob(["src/*.c"]),
    hdrs = glob(["src/*.h"]),
    copts = [
      "-std=gnu99",
      "-Wno-conversion",
      "-Wno-implicit-function-declaration",
      "-Wno-sign-conversion",
      "-Wno-switch-enum",
      "-Wno-unused-parameter",
    ],
    includes = ["src"],
)
