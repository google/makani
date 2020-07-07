load("//avionics/firmware:build_rules.bzl", "tms570_test")

tms570_test(
    name = "aio_test_application.elf",
    srcs = [
        "test_main.c",
    ],
    deps = [
        "//avionics/firmware/comms:tether_message_test",
        "//avionics/firmware/cpu:adc_test",
        "//avionics/firmware/cpu:io_test",
        "//avionics/firmware/cpu:memcpy_test",
        "//avionics/firmware/cpu:on_fatal_stub",
        "//avionics/firmware/drivers:spi_flash_test",
        "//avionics/firmware/identity",
        "//avionics/firmware/test:test_test",
        "//avionics/network:cvt_tms570s",
    ],
)
