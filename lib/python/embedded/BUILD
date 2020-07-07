package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library", "py_test")

# Python rules.

py_library(
    name = "common",
    srcs = [
        "build.py",
        "database.py",
        "hardware_db.py",
    ],
    deps = [
        "//lib/python:string_util",
    ],
)

py_binary(
    name = "process_commit",
    srcs = ["process_commit.py"],
    deps = [
        ":common",
        ":test_runner",
    ],
)

py_binary(
    name = "relay_device",
    srcs = ["relay_device.py"],
    deps = [
        requirement("pyserial"),
        requirement("pyudev"),
    ],
)

py_binary(
    name = "test_fixture",
    srcs = ["test_fixture.py"],
    deps = [
        ":common",
        ":relay_device",
        requirement("python-gflags"),
        requirement("PyYAML"),
    ],
)

py_binary(
    name = "test_runner",
    srcs = ["test_runner.py"],
    data = [
        "//avionics/bootloader:bootloader_client",
        "//avionics/linux:test_command",
    ],
    deps = [
        ":common",
        "//avionics/common:aio",
        "//avionics/common:h2py_pack_aio_header",
        requirement("pexpect"),
    ],
)

py_binary(
    name = "test_server",
    srcs = ["test_server.py"],
    deps = [
        ":common",
        ":test_fixture",
    ],
)

# Python tests.

py_test(
    name = "hardware_db_test",
    srcs = ["hardware_db_test.py"],
    deps = [
        ":common",
    ],
)

py_test(
    name = "test_fixture_test",
    srcs = ["test_fixture_test.py"],
    deps = [
        ":test_fixture",
        requirement("mock"),
    ],
)

py_test(
    name = "test_runner_test",
    srcs = ["test_runner_test.py"],
    deps = [
        ":test_fixture",
        ":test_runner",
        requirement("mock"),
        requirement("numpy"),
    ],
)
