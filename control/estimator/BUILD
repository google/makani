package(default_visibility = ["//visibility:public"])

load("//lib/bazel:c_rules.bzl", "makani_c_library", "makani_cc_test")

makani_c_library(
    name = "estimator_types",
    srcs = ["estimator_types.c"],
    hdrs = ["estimator_types.h"],
    deps = [
        "//avionics/common:plc_messages",
        "//common/c_math",
        "//control:sensor_types",
        "//control:system_types",
        "//control/experiments:experiment_types",
    ],
)

makani_c_library(
    name = "estimator_filter",
    srcs = [
        "estimator_filter.c",
    ],
    hdrs = [
        "estimator_filter.h",
    ],
    deps = [
        "//common:macros",
        "//common/c_math",
    ],
)

makani_c_library(
    name = "estimator_attitude",
    srcs = [
        "estimator_attitude.c",
    ],
    hdrs = [
        "estimator_attitude.h",
    ],
    nostatic_files = [
        "estimator_attitude.c",
    ],
    deps = [
        ":estimator_filter",
        ":estimator_types",
        "//common:macros",
        "//common/c_math",
        "//common/c_math:mahony_filter",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
        "//control/fault_detection:fault_detection_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "estimator_attitude_ground",
    srcs = [
        "estimator_attitude_ground.c",
    ],
    hdrs = [
        "estimator_attitude_ground.h",
    ],
    nostatic_files = [
        "estimator_attitude_ground.c",
    ],
    deps = [
        ":estimator_attitude",
        ":estimator_filter",
        ":estimator_types",
        "//common:macros",
        "//common/c_math",
        "//common/c_math:mahony_filter",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
        "//control/fault_detection:fault_detection_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "estimator_attitude_kite",
    srcs = [
        "estimator_attitude_kite.c",
    ],
    hdrs = [
        "estimator_attitude_kite.h",
    ],
    nostatic_files = [
        "estimator_attitude_kite.c",
    ],
    deps = [
        ":estimator_attitude",
        ":estimator_filter",
        ":estimator_types",
        "//common:macros",
        "//common/c_math",
        "//common/c_math:mahony_filter",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
        "//control/fault_detection:fault_detection_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "estimator_nav_ground",
    srcs = [
        "estimator_nav_ground.c",
    ],
    hdrs = [
        "estimator_nav_ground.h",
    ],
    nostatic_files = [
        "estimator_nav_ground.c",
    ],
    deps = [
        ":estimator_attitude_ground",
        ":estimator_nav",
        ":estimator_position_ground",
        ":estimator_types",
        "//common/c_math",
        "//control:common",
        "//control:ground_telemetry",
    ],
)

makani_c_library(
    name = "estimator_ground",
    srcs = [
        "estimator_ground.c",
    ],
    hdrs = [
        "estimator_ground.h",
    ],
    nostatic_files = [
        "estimator_ground.c",
    ],
    deps = [
        ":estimator_nav_ground",
        ":estimator_types",
        "//avionics/linux:aio",
        "//common/c_math",
        "//control:common",
        "//control:pack_ground_telemetry",
        "//control/avionics:avionics_interface",
    ],
)

makani_c_library(
    name = "estimator_nav",
    srcs = [
        "estimator_nav.c",
    ],
    hdrs = [
        "estimator_nav.h",
    ],
    deps = [
        ":estimator_types",
        "//common:macros",
        "//common/c_math",
        "//control:ground_frame",
        "//control/fault_detection:fault_detection_types",
    ],
)

makani_c_library(
    name = "estimator_nav_kite",
    srcs = [
        "estimator_nav_kite.c",
    ],
    hdrs = [
        "estimator_nav_kite.h",
    ],
    nostatic_files = [
        "estimator_nav_kite.c",
    ],
    deps = [
        ":estimator_attitude_kite",
        ":estimator_nav",
        ":estimator_position_kite",
        "//common:macros",
        "//common/c_math",
        "//control:control_telemetry",
        "//control:control_types",
        "//control:ground_frame",
        "//control/fault_detection:fault_detection_types",
    ],
)

makani_c_library(
    name = "estimator_position_ground",
    srcs = [
        "estimator_position_gps.c",
        "estimator_position_ground.c",
    ],
    hdrs = [
        "estimator_position_gps.h",
        "estimator_position_ground.h",
    ],
    nostatic_files = [
        "estimator_position_ground.c",
    ],
    deps = [
        ":estimator_filter",
        ":estimator_position_filter_ground",
        ":estimator_types",
        "//common:macros",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_util",
        "//control:ground_telemetry",
        "//control:system_types",
        "//control/fault_detection:fault_detection_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "estimator_position_kite",
    srcs = [
        "estimator_position_baro.c",
        "estimator_position_glas.c",
        "estimator_position_gps.c",
        "estimator_position_kite.c",
    ],
    hdrs = [
        "estimator_position_baro.h",
        "estimator_position_glas.h",
        "estimator_position_gps.h",
        "estimator_position_kite.h",
    ],
    nostatic_files = [
        "estimator_position_kite.c",
        "estimator_position_glas.c",
    ],
    deps = [
        ":estimator_filter",
        ":estimator_position_filter_kite",
        ":estimator_types",
        "//common:macros",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:system_types",
        "//control/fault_detection:fault_detection_types",
        "//system:labels",
    ],
)

makani_c_library(
    name = "estimator_position_filter",
    srcs = [
        "estimator_position_filter.c",
    ],
    hdrs = [
        "estimator_position_filter.h",
    ],
    deps = [
        ":estimator_filter",
        "//common/c_math",
        "//control:common",
        "//control:control_telemetry",
        "//control:control_util",
    ],
)

makani_c_library(
    name = "estimator_position_filter_ground",
    srcs = [
        "estimator_position_filter_ground.c",
    ],
    hdrs = [
        "estimator_position_filter_ground.h",
    ],
    deps = [
        ":estimator_position_filter",
        "//control:ground_telemetry",
    ],
)

makani_c_library(
    name = "estimator_position_filter_kite",
    srcs = [
        "estimator_position_filter_kite.c",
    ],
    hdrs = [
        "estimator_position_filter_kite.h",
    ],
    deps = [
        ":estimator_position_filter",
    ],
)

makani_c_library(
    name = "estimator",
    srcs = [
        "estimator.c",
        "estimator_apparent_wind.c",
        "estimator_encoders.c",
        "estimator_experiment.c",
        "estimator_ground_station.c",
        "estimator_joystick.c",
        "estimator_nav_kite.c",
        "estimator_perch_azi.c",
        "estimator_tether_anchor.c",
        "estimator_tether_force.c",
        "estimator_tether_ground_angles.c",
        "estimator_weather.c",
        "estimator_winch.c",
        "estimator_wind.c",
    ],
    hdrs = [
        "estimator.h",
        "estimator_apparent_wind.h",
        "estimator_encoders.h",
        "estimator_experiment.h",
        "estimator_ground_station.h",
        "estimator_joystick.h",
        "estimator_nav_kite.h",
        "estimator_perch_azi.h",
        "estimator_tether_anchor.h",
        "estimator_tether_force.h",
        "estimator_tether_ground_angles.h",
        "estimator_weather.h",
        "estimator_winch.h",
        "estimator_wind.h",
    ],
    nostatic_files = [
        "estimator.c",
        "estimator_apparent_wind.c",
        "estimator_filter.c",
        "estimator_ground_station.c",
        "estimator_joystick.c",
        "estimator_perch_azi.c",
        "estimator_wind.c",
    ],
    deps = [
        ":estimator_attitude_kite",
        ":estimator_filter",
        ":estimator_nav_kite",
        ":estimator_position_kite",
        ":estimator_types",
        "//avionics/common:novatel_types",
        "//common:macros",
        "//common/c_math",
        "//control:common",
        "//control:control_params",
        "//control:control_telemetry",
        "//control:control_util",
        "//control:sensor_types",
        "//control:system_types",
        "//control/experiments:experiment_util",
        "//control/fault_detection:fault_detection_types",
        "//system:labels",
    ],
)

makani_cc_test(
    name = "estimator_apparent_wind_test",
    srcs = ["estimator_apparent_wind_test.cc"],
    deps = [
        ":estimator_nostatic",
        "//control/fault_detection",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "estimator_attitude_kite_test",
    srcs = ["estimator_attitude_kite_test.cc"],
    deps = [
        ":estimator_attitude_kite_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "estimator_attitude_ground_test",
    srcs = ["estimator_attitude_ground_test.cc"],
    deps = [
        ":estimator_attitude_ground_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "estimator_ground_station_test",
    srcs = ["estimator_ground_station_test.cc"],
    deps = [
        ":estimator_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "estimator_nav_kite_test",
    srcs = ["estimator_nav_kite_test.cc"],
    deps = [
        ":estimator_nav_kite_nostatic",
        "//control/fault_detection",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "estimator_nav_test",
    srcs = ["estimator_nav_test.cc"],
    deps = [
        ":estimator_nav",
        "//control/fault_detection",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "estimator_joystick_test",
    srcs = ["estimator_joystick_test.cc"],
    deps = [
        ":estimator_nostatic",
        "//control/fault_detection",
        "//lib/util:test_util",
        "@usr_lib//:glog",
    ],
)

makani_cc_test(
    name = "estimator_position_filter_test",
    srcs = ["estimator_position_filter_test.cc"],
    deps = [
        ":estimator_position_filter_kite",
        ":estimator_position_kite_nostatic",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "estimator_position_glas_test",
    srcs = ["estimator_position_glas_test.cc"],
    deps = [
        ":estimator_position_kite_nostatic",
    ],
)

makani_cc_test(
    name = "estimator_test",
    srcs = ["estimator_test.cc"],
    deps = [
        ":estimator_nostatic",
        "//control/fault_detection",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "estimator_tether_force_test",
    srcs = ["estimator_tether_force_test.cc"],
    deps = [
        ":estimator_nostatic",
        "//control/fault_detection",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "estimator_weather_test",
    srcs = ["estimator_weather_test.cc"],
    deps = [
        ":estimator_nostatic",
        "//control/fault_detection",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "estimator_wind_test",
    srcs = ["estimator_wind_test.cc"],
    deps = [
        ":estimator_nostatic",
        "//control/fault_detection",
        "//lib/util:test_util",
    ],
)
