load("//avionics/firmware:build_rules.bzl", "tms570_application")

tms570_application(
    name = "imu_test_application.elf",
    srcs = [
        "imu_test.c",
    ],
    deps = [
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/drivers:adis16488_test",
        "//avionics/firmware/identity",
        "//avionics/firmware/test",
        "//avionics/network:cvt_flight_computer",
    ],
)
