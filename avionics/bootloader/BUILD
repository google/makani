package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library", "py_test")

# Python/h2py rules.

py_binary(
    name = "bootloader_client",
    srcs = ["bootloader_client.py"],
    tags = ["tms570_util"],
    deps = [
        ":generate_image_library",
        "//avionics/bootloader/firmware:h2py_update_server",
        "//avionics/firmware/identity:h2py_identity_types",
        "//avionics/network:network_config",
        "//lib/python:c_helpers",
    ],
)

py_binary(
    name = "fake_bootloader_client",
    srcs = ["fake_bootloader_client.py"],
    tags = ["tms570_util"],
    deps = [":bootloader_client"],
)

py_binary(
    name = "generate_image",
    srcs = ["generate_image.py"],
    tags = ["tms570_util"],
    deps = [
        ":generate_image_library",
    ],
)

py_library(
    name = "generate_image_library",
    srcs = ["generate_image.py"],
    data = [
        "//third_party_toolchains/gcc_arm_none_eabi:everything",
    ],
    deps = [
        "//avionics/firmware/identity:h2py_identity_types",
        "//avionics/firmware/startup:h2py_pack_ldscript_types",
        "//avionics/network:network_config",
        "//lib/python:c_helpers",
    ],
)

py_library(
    name = "system_config",
    srcs = ["system_config.py"],
    data = ["system_config.yaml"],
    deps = [
        "//avionics/network:network_config",
        "//control:h2py_system_types",
        "//lib/python:c_helpers",
    ],
)

py_test(
    name = "system_config_test",
    srcs = ["system_config_test.py"],
    deps = [
        ":system_config",
        "//config:mconfig",
        "//control:h2py_system_types",
    ],
)

py_binary(
    name = "jlink_client",
    srcs = ["jlink_client.py"],
    data = ["jlink_tms570_settings.txt"],
    tags = ["tms570_util"],
    deps = [
        "//avionics/bootloader:generate_image",
        "//avionics/common:aio",
        "//avionics/firmware/params:param_util",
        "//avionics/linux/provision:process_helper",
        "//lib/python:c_helpers",
    ],
)

py_binary(
    name = "parallel_bootloader",
    srcs = ["parallel_bootloader.py"],
    tags = ["tms570_util"],
    deps = [
        "//avionics/network:network_util",
    ],
)

py_binary(
    name = "program",
    srcs = ["program.py"],
    data = [
        ":bootloader_client",
        "//avionics/firmware/serial:serial_params_templates",
    ],
    tags = ["tms570_util"],
    deps = [
        ":jlink_client",
        ":parallel_bootloader",
        ":system_config",
        "//avionics/firmware/params:param_util",
        "//avionics/network:h2py_aio_node",
        "//avionics/network:network_util",
        "//avionics/network:node_locations",
        "//lib/bazel:bazel_util",
    ],
)

py_binary(
    name = "program_ui",
    srcs = ["program_ui.py"],
    tags = ["tms570_util"],
    deps = [
        ":program",
        "//avionics/common:aio",
        "//avionics/linux/provision:process_helper",
        "//avionics/linux/provision:ui_helper",
        "//lib/bazel:bazel_util",
    ],
)

# Python tests.

py_test(
    name = "bootloader_client_test",
    srcs = ["bootloader_client_test.py"],
    deps = [
        ":bootloader_client",
        "//avionics/common:aio",
        "//avionics/network:h2py_aio_labels",
        "//avionics/network:h2py_aio_node",
        "//system:h2py_labels",
        requirement("mock"),
    ],
)

py_test(
    name = "program_test",
    srcs = ["program_test.py"],
    deps = [
        ":program",
        requirement("mock"),
    ],
)

py_test(
    name = "program_ui_test",
    srcs = ["program_ui_test.py"],
    deps = [
        ":program_ui",
        requirement("mock"),
    ],
)
