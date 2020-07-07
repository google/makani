package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "liblabjackusb.so",
  srcs = [
      "LabjackExodriver/labjackusb.h",
      "LabjackExodriver/labjackusb.c",
  ],
  linkopts = [
      "-lc",
      "-lusb-1.0",
  ],
  linkshared = 1,
  copts = [
      "--std=c99",
      "-Wno-conversion",
      "-Wno-switch-default",
      "-Wno-cast-qual",
  ],
)

filegroup(
  name = "labjack_python",
  srcs = [
      "__init__.py",
      "liblabjackusb.so",
      "LabjackPython/__init__.py",
      "LabjackPython/LabJackPython.py",
      "LabjackPython/Modbus.py",
      "LabjackPython/u3.py",
      "LabjackPython/u6.py",
  ],
)
