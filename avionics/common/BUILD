package(default_visibility = ["//visibility:public"])

load("@python_pip//:requirements.bzl", "requirement")
load("//avionics/firmware:build_rules.bzl", "configurable_macro")
load(
    "//lib/bazel:autogen_rules.bzl",
    "h2py_library",
    "message_ruleset",
)
load("//lib/bazel:c_rules.bzl", "makani_c_library", "makani_cc_test")
load("//lib/bazel:py_rules.bzl", "py_binary", "py_library", "py_test")
load("//lib/python/pack2:build_rules.bzl", "pack2")

exports_files(["generate_cvt_lib.py"])

# C libraries.

makani_c_library(
    name = "adis16488_types",
    hdrs = ["adis16488_types.h"],
)

message_ruleset(
    name = "aio_header",
    generate_cvt_lib = False,
    generate_h2py_lib = True,
    header = "aio_header.h",
    source = "aio_header.c",
)

message_ruleset(
    name = "avionics_messages",
    header = "avionics_messages.h",
    deps = [
        ":actuator_types",
        ":ars308_types",
        ":batt_types",
        ":build_info_types",
        ":decawave_types",
        ":eop_messages",
        ":faa_light_types",
        ":faults",
        ":gill_types",
        ":gps_types",
        ":imu_types",
        ":loadcell_types",
        ":motor_adc_defines",
        ":motor_angle_types",
        ":motor_foc_types",
        ":motor_profiler_types",
        ":motor_thermal_types",
        ":mvlv_types",
        ":network_config",
        ":network_diag_types",
        ":novatel_types",
        ":plc_messages",
        ":plc_types",
        ":septentrio_types",
        ":servo_types",
        ":short_stack_types",
        ":winch_messages",
        "//avionics/firmware/drivers:microhard_types",
        "//avionics/firmware/identity:identity_types",
        "//avionics/firmware/monitors:aio_types",
        "//avionics/firmware/monitors:batt_types",
        "//avionics/firmware/monitors:cs_types",
        "//avionics/firmware/monitors:fc_types",
        "//avionics/firmware/monitors:ground_io_types",
        "//avionics/firmware/monitors:ina219_types",
        "//avionics/firmware/monitors:joystick_types",
        "//avionics/firmware/monitors:loadcell_types",
        "//avionics/firmware/monitors:mcp9800_types",
        "//avionics/firmware/monitors:mvlv_types",
        "//avionics/firmware/monitors:recorder_types",
        "//avionics/firmware/monitors:servo_types",
        "//avionics/firmware/monitors:short_stack_types",
        "//avionics/firmware/monitors:si7021_types",
        "//avionics/firmware/network:net_mon_types",
        "//avionics/firmware/params:param_types",
        "//avionics/firmware/serial:serial_params_c",
        "//avionics/motor/monitors:types",
        "//avionics/network:aio_labels",
        "//common/c_math:common",
        "//control/experiments:experiment_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "aio_version",
    srcs = ["aio_version.c"],
    hdrs = ["aio_version.h"],
    deps = [":aio_version_code"],
)

# aio_version_code.h is auto-generated. See the genrule below.
makani_c_library(
    name = "aio_version_code",
    hdrs = ["aio_version_code.h"],
)

makani_c_library(
    name = "actuator_types",
    hdrs = ["actuator_types.h"],
)

makani_c_library(
    name = "ars308_types",
    hdrs = ["ars308_types.h"],
)

makani_c_library(
    name = "batt_types",
    hdrs = ["batt_types.h"],
)

makani_c_library(
    name = "bits",
    srcs = ["bits.c"],
    hdrs = ["bits.h"],
    deps = [
        ":endian",
        "//common/c_math:common",
    ],
)

makani_c_library(
    name = "build_info",
    srcs = [
        "build_info.c",
        "generated_build_info.h",
    ],
    hdrs = [
        "build_info.h",
        "build_info_types.h",
    ],
    deps = [
        ":strings",
        "//common:macros",
    ],
)

makani_c_library(
    name = "build_info_types",
    hdrs = ["build_info_types.h"],
)

h2py_library(
    name = "h2py_build_info_types",
    header = "build_info_types.h",
)

h2py_library(
    name = "h2py_build_info",
    function_deps = [":build_info"],
    header = "build_info.h",
)

makani_c_library(
    name = "coning_sculling",
    srcs = ["coning_sculling.c"],
    hdrs = ["coning_sculling.h"],
    nostatic_files = ["coning_sculling.c"],
    deps = ["//common/c_math:common"],
)

makani_c_library(
    name = "controller_arbitration",
    srcs = ["controller_arbitration.c"],
    hdrs = ["controller_arbitration.h"],
    nostatic_files = ["controller_arbitration.c"],
    deps = [
        ":cvt_avionics_messages",
        "//avionics/network:aio_labels",
    ],
)

makani_c_library(
    name = "crc",
    srcs = ["crc.c"],
    hdrs = ["crc.h"],
)

cvt_deps = [
    ":cvt_entries",
    ":pack_avionics_messages",
    "//avionics/network:aio_node",
    "//avionics/network:message_type",
]

makani_c_library(
    name = "cvt",
    srcs = [
        "aio_header.h",
        "cvt.c",
    ],
    hdrs = [
        "cvt.h",
    ],
    undefined_symbols = [
        "GetCvtEntry",  # Defined by application.
    ],
    deps = select({
        "//lib/bazel:tms570_mode": cvt_deps + [
            "//avionics/firmware/cpu:peripherals",
        ],
        "//conditions:default": cvt_deps,
    }),
)

makani_c_library(
    name = "cvt_entries",
    hdrs = [
        "cvt_entries.h",
    ],
    deps = [
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
    ],
)

makani_c_library(
    name = "debounce",
    srcs = ["debounce.c"],
    hdrs = ["debounce.h"],
)

makani_c_library(
    name = "decawave_types",
    hdrs = ["decawave_types.h"],
)

makani_c_library(
    name = "encoder_types",
    hdrs = ["encoder_types.h"],
)

makani_c_library(
    name = "endian",
    srcs = ["endian.c"],
    hdrs = ["endian.h"],
)

message_ruleset(
    name = "eop_messages",
    generate_cvt_lib = False,
    header = "eop_messages.h",
)

# eop_version.h is auto-generated. See the genrule below.
makani_c_library(
    name = "eop_version",
    hdrs = ["eop_version.h"],
)

makani_c_library(
    name = "faa_light_types",
    hdrs = ["faa_light_types.h"],
)

makani_c_library(
    name = "faults",
    srcs = ["faults.c"],
    hdrs = ["faults.h"],
    deps = [":bits"],
)

makani_c_library(
    name = "gill",
    srcs = [
        "gill_ascii.c",
        "gill_binary.c",
        "gill_serial.c",
    ],
    hdrs = [
        "gill_ascii.h",
        "gill_binary.h",
        "gill_serial.h",
        "gill_types.h",
    ],
    deps = [
        ":endian",
        ":nmea",
        ":serial_parse",
        ":strings",
        "//avionics/common/fast_math",
        "//common:macros",
    ],
)

makani_c_library(
    name = "gill_types",
    hdrs = ["gill_types.h"],
)

makani_c_library(
    name = "gps_parse",
    srcs = ["gps_parse.c"],
    hdrs = ["gps_parse.h"],
    deps = [
        ":endian",
        ":gps_types",
    ],
)

makani_c_library(
    name = "gps_types",
    hdrs = ["gps_types.h"],
)

makani_c_library(
    name = "imu_output",
    srcs = ["imu_output.c"],
    hdrs = ["imu_output.h"],
    defines = (configurable_macro("CONING_SCULLING_DECIMATION", 1, 100, 24) +
               configurable_macro("IMU_OUTPUT_DECIMATION", 1, 10, 4)),
    deps = [
        ":coning_sculling",
        ":imu_config_c",
        "//avionics/common:cvt_avionics_messages",
        "//avionics/firmware/drivers:adis16488",
        "//avionics/firmware/network:net_mon",
    ],
)

makani_c_library(
    name = "imu_types",
    hdrs = ["imu_types.h"],
)

makani_c_library(
    name = "loadcell_types",
    hdrs = ["loadcell_types.h"],
)

makani_c_library(
    name = "motor_adc_defines",
    hdrs = ["motor_adc_defines.h"],
)

makani_c_library(
    name = "motor_angle_types",
    hdrs = ["motor_angle_types.h"],
)

makani_c_library(
    name = "motor_foc_types",
    hdrs = ["motor_foc_types.h"],
)

makani_c_library(
    name = "motor_profiler_types",
    hdrs = ["motor_profiler_types.h"],
)

makani_c_library(
    name = "motor_thermal_types",
    hdrs = ["motor_thermal_types.h"],
)

makani_c_library(
    name = "motor_util",
    srcs = ["motor_util.c"],
    hdrs = ["motor_util.h"],
    deps = [
        "//avionics/network:aio_labels",
    ],
)

h2py_library(
    name = "h2py_motor_util",
    function_deps = [":motor_util"],
    header = "motor_util.h",
)

makani_c_library(
    name = "mvlv_types",
    hdrs = ["mvlv_types.h"],
)

makani_c_library(
    name = "network_addresses",
    hdrs = ["network_addresses.h"],
)

makani_c_library(
    name = "network_config",
    srcs = ["network_config.c"],
    hdrs = ["network_config.h"],
    deps = [
        ":network_addresses",
        ":plc_messages",
        "//avionics/network:aio_node",
        "//avionics/network:eop_message_type",
        "//avionics/network:message_type",
        "//avionics/network:switch_def",
        "//avionics/network:winch_message_type",
    ],
)

makani_c_library(
    name = "network_diag_types",
    hdrs = ["network_diag_types.h"],
    deps = [
        ":network_addresses",
    ],
)

makani_c_library(
    name = "novatel",
    srcs = [
        "novatel_binary.c",
        "novatel_serial.c",
        "novatel_serial.h",
    ],
    hdrs = [
        "novatel_binary.h",
        "novatel_types.h",
    ],
    deps = [
        ":crc",
        ":endian",
        ":gps_parse",
        ":novatel_types",
        ":rtcm3",
        ":serial_parse",
        ":strings",
        "//common:macros",
        "//common/c_math:common",
    ],
)

makani_c_library(
    name = "novatel_types",
    srcs = ["novatel_types.c"],
    hdrs = ["novatel_types.h"],
    deps = [":gps_types"],
)

makani_c_library(
    name = "nmea",
    srcs = ["nmea.c"],
    hdrs = ["nmea.h"],
    deps = [
        ":serial_parse",
        ":strings",
    ],
)

makani_c_library(
    name = "pack_tether_message",
    srcs = [
        "aio_version.h",
        "pack_tether_message.c",
    ],
    hdrs = ["pack_tether_message.h"],
    undefined_symbols = ["GetAioVersion"],
    deps = [
        ":avionics_messages",
        ":bits",
        ":crc",
        ":safety_codes",
        ":tether_message_types",
        ":tether_op",
        "//avionics/common/fast_math",
        "//common:macros",
    ],
)

makani_c_library(
    name = "pitot_cover_types",
    hdrs = ["pitot_cover_types.h"],
)

message_ruleset(
    name = "plc_messages",
    generate_cvt_lib = False,
    header = "plc_messages.h",
    deps = [
        ":faults",
    ],
)

makani_c_library(
    name = "plc_types",
    hdrs = ["plc_types.h"],
)

makani_c_library(
    name = "rtcm3",
    srcs = ["rtcm3.c"],
    hdrs = ["rtcm3.h"],
    deps = [
        ":crc",
        ":endian",
        ":serial_parse",
    ],
)

makani_c_library(
    name = "safety_codes",
    hdrs = [
        "safety_codes.h",
    ],
)

makani_c_library(
    name = "septentrio",
    srcs = [
        "septentrio_sbf.c",
        "septentrio_serial.c",
    ],
    hdrs = [
        "septentrio_sbf.h",
        "septentrio_serial.h",
    ],
    deps = [
        ":crc",
        ":endian",
        ":rtcm3",
        ":septentrio_types",
        ":serial_parse",
        "//common:macros",
    ],
)

makani_c_library(
    name = "septentrio_types",
    srcs = ["septentrio_types.c"],
    hdrs = ["septentrio_types.h"],
    deps = [":gps_types"],
)

makani_c_library(
    name = "serial_parse",
    srcs = ["serial_parse.c"],
    hdrs = ["serial_parse.h"],
    deps = select({
        "//lib/bazel:tms570_mode": [
            "//avionics/firmware/cpu:peripherals",
            "//common:macros",
        ],
        "//conditions:default": ["//common:macros"],
    }),
)

makani_c_library(
    name = "servo_types",
    hdrs = ["servo_types.h"],
    deps = [
        ":actuator_types",
        ":faults",
    ],
)

makani_c_library(
    name = "short_stack_types",
    hdrs = ["short_stack_types.h"],
)

makani_c_library(
    name = "strings",
    srcs = ["strings.c"],
    hdrs = ["strings.h"],
    deps = ["//common:macros"],
)

makani_c_library(
    name = "tether_convert",
    srcs = ["tether_convert.c"],
    hdrs = ["tether_convert.h"],
    deps = [
        ":gill_types",
        ":novatel_types",
        ":tether_message_types",
        "//avionics/motor/firmware:flags",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//avionics/network:message_stats",
    ],
)

makani_c_library(
    name = "tether_cvt",
    srcs = [
        "tether_cvt.c",
        "tether_cvt_op.c",
        "tether_cvt_rtcm.c",
    ],
    hdrs = [
        "tether_cvt.h",
        "tether_cvt_op.h",
        "tether_cvt_rtcm.h",
    ],
    deps = [
        ":controller_arbitration",
        ":cvt_avionics_messages",
        ":safety_codes",
        ":tether_convert",
        ":tether_message_types",
        ":tether_op",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//common:macros",
    ],
)

makani_c_library(
    name = "tether_message",
    srcs = ["tether_message.c"],
    hdrs = ["tether_message.h"],
    nostatic_files = ["tether_message.c"],
    deps = [
        ":tether_convert",
        ":tether_cvt",
        ":tether_message_types",
        "//avionics/network:aio_labels",
        "//avionics/network:aio_node",
        "//common:macros",
    ],
)

makani_c_library(
    name = "tether_message_types",
    hdrs = ["tether_message_types.h"],
    deps = [
        "//avionics/common:avionics_messages",
        "//avionics/network:aio_node",
    ],
)

makani_c_library(
    name = "tether_op",
    srcs = ["tether_op.c"],
    hdrs = ["tether_op.h"],
    deps = [
        ":avionics_messages",
        "//common:macros",
        "//common:ring",
    ],
)

message_ruleset(
    name = "winch_messages",
    generate_cvt_lib = False,
    header = "winch_messages.h",
    deps = [
        ":faults",
        "//system:labels",
    ],
)

# Code generation.

py_binary(
    name = "generate_version_crc",
    srcs = ["generate_version_crc.py"],
    deps = [
        requirement("python-gflags"),
    ],
)

genrule(
    name = "aio_version_code_genrule",
    srcs = [
        "aio_header.h",
        "generate_version_crc.py",
        "network_config.c",
        "network_config.h",
        "pack_aio_header.c",
        "pack_aio_header.h",
        "pack_avionics_messages.c",
        "pack_avionics_messages.h",
        "pack_eop_messages.c",
        "pack_eop_messages.h",
        "pack_plc_messages.c",
        "pack_plc_messages.h",
        "pack_tether_message.c",
        "pack_tether_message.h",
        "pack_winch_messages.c",
        "pack_winch_messages.h",
        "tether_message_types.h",
        "tether_op.h",
        "//avionics/network:aio_node.h",
        "//avionics/network:message_type.h",
        "//avionics/network:routes.c",
        "//avionics/network:routes.h",
        "//control:pack_ground_telemetry.c",
        "//control:pack_ground_telemetry.h",
        "//control:pack_control_telemetry.c",
        "//control:pack_control_telemetry.h",
        "//gs/aio_snapshot:pack_aio_telemetry.c",
        "//gs/aio_snapshot:pack_aio_telemetry.h",
        "//sim:pack_sim_messages.c",
        "//sim:pack_sim_messages.h",
        "//sim:pack_sim_telemetry.c",
        "//sim:pack_sim_telemetry.h",
    ],
    outs = ["aio_version_code.h"],
    cmd = " ".join([
        "$(location :generate_version_crc)",
        "--autogen_root=$(GENDIR)",
        "--output_file=$(location aio_version_code.h)",
        "--macro_name=AIO_VERSION",
        "--bits=16",
        "$(SRCS)",
        "$@",
    ]),
    tools = [":generate_version_crc"],
)

genrule(
    name = "eop_version_genrule",
    srcs = [
        "eop_messages.h",
        "generate_version_crc.py",
        "pack_eop_messages.c",
        "pack_eop_messages.h",
    ],
    outs = ["eop_version.h"],
    cmd = " ".join([
        "$(location :generate_version_crc)",
        "--autogen_root=$(GENDIR)",
        "--output_file=$(location eop_version.h)",
        "--macro_name=EOP_VERSION",
        "--bits=32",
        "$(SRCS)",
        "$@",
    ]),
    tools = [":generate_version_crc"],
)

h2py_library(
    name = "h2py_actuator_types",
    header = "actuator_types.h",
    header_deps = [":actuator_types"],
)

h2py_library(
    name = "h2py_aio_header",
    header = "aio_header.h",
    header_deps = [":aio_header"],
)

h2py_library(
    name = "h2py_aio_version",
    header = "aio_version_code.h",
    header_deps = [":aio_version_code"],
)

h2py_library(
    name = "h2py_batt_types",
    header = "batt_types.h",
    header_deps = [":batt_types"],
)

h2py_library(
    name = "h2py_coning_sculling",
    function_deps = [
        ":coning_sculling",
    ],
    header = "coning_sculling.h",
    header_deps = [
        "//common/c_math:common",
    ],
    visibility = ["//visibility:public"],
)

h2py_library(
    name = "h2py_cvt",
    function_deps = [
        ":cvt",
        "//avionics/network:cvt_all",
    ],
    header = "cvt.h",
    header_deps = [
        ":avionics_messages",
        "//avionics/network:aio_node",
        "//avionics/network:message_type",
    ],
)

h2py_library(
    name = "h2py_encoder_types",
    function_deps = [":encoder_types"],
    header = "encoder_types.h",
)

h2py_library(
    name = "h2py_faults",
    header = "faults.h",
    header_deps = [":faults"],
)

h2py_library(
    name = "h2py_loadcell_types",
    header = "loadcell_types.h",
    header_deps = [":loadcell_types"],
)

h2py_library(
    name = "h2py_motor_thermal_types",
    header = "motor_thermal_types.h",
    header_deps = [":motor_thermal_types"],
)

h2py_library(
    name = "h2py_motor_adc_defines",
    header = "motor_adc_defines.h",
    header_deps = [":motor_adc_defines"],
)

h2py_library(
    name = "h2py_mvlv_types",
    header = "mvlv_types.h",
    header_deps = [":mvlv_types"],
)

h2py_library(
    name = "h2py_novatel_types",
    header = "novatel_types.h",
    header_deps = [":novatel_types"],
)

h2py_library(
    name = "h2py_network_config",
    function_deps = [":network_config"],
    header = "network_config.h",
)

h2py_library(
    name = "h2py_pitot_cover_types",
    header = "pitot_cover_types.h",
    header_deps = [":pitot_cover_types"],
)

h2py_library(
    name = "h2py_plc_messages",
    header = "plc_messages.h",
    header_deps = [":plc_messages"],
)

h2py_library(
    name = "h2py_safety_codes",
    header = "safety_codes.h",
    header_deps = [":safety_codes"],
)

h2py_library(
    name = "h2py_septentrio_types",
    header = "septentrio_types.h",
    header_deps = [":septentrio_types"],
)

h2py_library(
    name = "h2py_servo_types",
    header = "servo_types.h",
    header_deps = [":servo_types"],
)

h2py_library(
    name = "h2py_tether_message_types",
    header = "tether_message_types.h",
    header_deps = [":tether_message_types"],
)

h2py_library(
    name = "h2py_winch_messages",
    header = "winch_messages.h",
    header_deps = [":winch_messages"],
)

py_library(
    name = "aio",
    srcs = ["aio.py"],
    srcs_version = "PY2AND3",
    deps = [
        ":h2py_aio_version",
        ":h2py_cvt",
        ":h2py_network_config",
        ":h2py_pack_aio_header",
        ":h2py_pack_avionics_messages",
        "//avionics/network:h2py_aio_node",
        "//avionics/network:h2py_aio_node_to_ip_address",
        "//avionics/network:h2py_message_type",
        "//control:h2py_pack_control_telemetry",
        "//control:h2py_pack_ground_telemetry",
        "//lib/python:c_helpers",
        "//sim:h2py_pack_sim_messages",
        "//sim:h2py_pack_sim_telemetry",
    ],
)

py_library(
    name = "cmd_client",
    srcs = ["cmd_client.py"],
    deps = [
        ":aio",
        "//avionics/common:h2py_pack_avionics_messages",
    ],
)

# Pack2 libraries.

pack2(
    name = "gps_receiver",
    src = "gps_receiver.p2",
)

pack2(
    name = "imu_config",
    src = "imu_config.p2",
)

# Unit tests.

py_test(
    name = "aio_test",
    srcs = ["aio_test.py"],
    tags = ["exclusive"],
    deps = [
        ":aio",
    ],
)

makani_cc_test(
    name = "bits_test",
    srcs = ["bits_test.cc"],
    deps = [
        ":bits",
        ":endian",
        "//common:macros",
        "//common/c_math:common",
    ],
)

makani_cc_test(
    name = "coning_sculling_test",
    srcs = ["coning_sculling_test.cc"],
    deps = [
        ":coning_sculling_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "controller_arbitration_test",
    srcs = ["controller_arbitration_test.cc"],
    deps = [
        ":controller_arbitration_nostatic",
        "//avionics/network:cvt_all",
    ],
)

makani_cc_test(
    name = "debounce_test",
    srcs = ["debounce_test.cc"],
    deps = [":debounce"],
)

makani_cc_test(
    name = "endian_test",
    srcs = ["endian_test.cc"],
    deps = [
        ":endian",
        "//common:macros",
    ],
)

makani_cc_test(
    name = "gill_serial_test",
    srcs = ["gill_serial_test.cc"],
    deps = [":gill"],
)

makani_cc_test(
    name = "gps_parse_test",
    srcs = ["gps_parse_test.cc"],
    deps = [
        ":gps_parse",
        "//common:macros",
    ],
)

makani_cc_test(
    name = "nmea_test",
    srcs = ["nmea_test.cc"],
    deps = [":nmea"],
)

makani_cc_test(
    name = "novatel_serial_test",
    srcs = [
        "novatel_serial.h",
        "novatel_serial_test.cc",
    ],
    deps = ["novatel"],
)

makani_cc_test(
    name = "pack_tether_message_test",
    srcs = ["pack_tether_message_test.cc"],
    deps = [
        ":aio_version",
        ":avionics_messages",
        ":pack_tether_message",
        "//common/c_math:common",
        "//control/crosswind:crosswind_types",
        "//control/fault_detection:fault_detection_types",
        "//control/hover:hover_types",
        "//control/trans_in:trans_in_types",
    ],
)

makani_cc_test(
    name = "rtcm3_test",
    srcs = ["rtcm3_test.cc"],
    deps = [
        ":rtcm3",
        "//common:macros",
    ],
)

makani_cc_test(
    name = "septentrio_serial_test",
    srcs = ["septentrio_serial_test.cc"],
    deps = [":septentrio"],
)

makani_cc_test(
    name = "serial_parse_test",
    srcs = ["serial_parse_test.cc"],
    deps = [
        ":serial_parse",
        "//common:macros",
    ],
)

makani_cc_test(
    name = "strings_test",
    srcs = ["strings_test.cc"],
    deps = [":strings"],
)

makani_cc_test(
    name = "tether_message_test",
    srcs = ["tether_message_test.cc"],
    deps = [
        ":pack_tether_message",
        ":tether_message_nostatic",
        "//avionics/network:cvt_all",
        "//common:macros",
    ],
)
