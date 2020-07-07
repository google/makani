package(default_visibility = ["//visibility:public"])

load("//lib/bazel:c_rules.bzl", "makani_cc_library", "makani_cc_test")

makani_cc_library(
    name = "model",
    srcs = ["model.cc"],
    hdrs = ["model.h"],
    deps = [
        "//sim:faults",
        "//sim:interfaces",
        "//sim:sim_types",
        "//sim:state",
    ],
)

makani_cc_library(
    name = "actuator",
    hdrs = ["actuators/actuator.h"],
    deps = [
        ":model",
        "//avionics/common:avionics_messages",
        "//common:macros",
        "//sim:sim_messages",
    ],
)

makani_cc_library(
    name = "rigid_body",
    hdrs = ["rigid_bodies/rigid_body.h"],
    deps = [
        ":model",
        "//common:macros",
    ],
)

makani_cc_library(
    name = "sensor",
    hdrs = ["sensors/sensor.h"],
    deps = [
        ":model",
        "//common:macros",
        "//sim:sim_messages",
    ],
)

MODEL_DEPS = [
    "//avionics/common:cvt_avionics_messages",
    "//avionics/common:gps_receiver_c",
    "//avionics/common:motor_util",
    "//avionics/common:safety_codes",
    "//avionics/common:tether_message_types",
    "//avionics/linux:aio",
    "//avionics/linux:clock",
    "//avionics/linux:cvt_util",
    "//avionics/motor/firmware:flags",
    "//avionics/network:aio_labels",
    "//common/c_math:c_math",
    "//common:macros",
    "//common:runfiles_dir",
    "//control/avionics:avionics_interface_types",
    "//control:control_util",
    "//sim/math:math",
    "//sim/physics:physics",
    "//sim/physics:ground_frame",
    "//sim/physics:wind_frame",
    "//sim:faults",
    "//sim:interfaces",
    "//sim:sim_telemetry",
    "//sim:sim_types",
    "//sim:state",
    "//system:labels",
    "@usr_lib//:gflags",
    "@usr_lib//:glog",
    ":model",
]

ACTUATOR_DEPS = MODEL_DEPS + [":actuator"]

RIGID_BODY_DEPS = MODEL_DEPS + [":rigid_body"]

SENSOR_DEPS = MODEL_DEPS + [":sensor"]

makani_cc_library(
    name = "base_system_model",
    srcs = ["base_system_model.cc"],
    hdrs = ["base_system_model.h"],
    deps = SENSOR_DEPS,
)

makani_cc_library(
    name = "buoy",
    srcs = ["rigid_bodies/buoy.cc"],
    hdrs = ["rigid_bodies/buoy.h"],
    deps = RIGID_BODY_DEPS + [
        ":environment",
        ":sea",
    ],
)

makani_cc_library(
    name = "delayed_signal",
    srcs = ["signals/delayed_signal.cc"],
    hdrs = ["signals/delayed_signal.h"],
    deps = MODEL_DEPS,
)

makani_cc_library(
    name = "environment",
    srcs = ["environment.cc"],
    hdrs = ["environment.h"],
    deps = MODEL_DEPS,
)

makani_cc_library(
    name = "ground_station_v2",
    srcs = ["actuators/ground_station_v2.cc"],
    hdrs = ["actuators/ground_station_v2.h"],
    deps = ACTUATOR_DEPS + [
        ":ground_station_util",
        "//control:vessel_frame",
    ],
)

makani_cc_library(
    name = "ground_station_util",
    srcs = ["ground_station_util.cc"],
    hdrs = ["ground_station_util.h"],
    deps = [
        "//common/c_math",
        "//control:system_params",
        "//sim:sim_types",
        "//sim/physics",
    ],
)

makani_cc_library(
    name = "high_voltage_harness",
    srcs = ["high_voltage_harness.cc"],
    hdrs = ["high_voltage_harness.h"],
    deps = MODEL_DEPS,
)

makani_cc_library(
    name = "imu_mount",
    srcs = ["imu_mount.cc"],
    hdrs = ["imu_mount.h"],
    deps = MODEL_DEPS,
)

makani_cc_library(
    name = "measurement",
    srcs = ["signals/measurement.cc"],
    hdrs = ["signals/measurement.h"],
    deps = MODEL_DEPS,
)

makani_cc_library(
    name = "perch",
    srcs = ["perch.cc"],
    hdrs = ["perch.h"],
    deps = MODEL_DEPS + [
        ":buoy",
        ":ground_station_util",
    ],
)

makani_cc_library(
    name = "platform",
    srcs = ["rigid_bodies/platform.cc"],
    hdrs = ["rigid_bodies/platform.h"],
    deps = RIGID_BODY_DEPS + [
        ":buoy",
        ":ground_station_v2",
    ],
)

makani_cc_library(
    name = "power_sys",
    srcs = ["power_sys.cc"],
    hdrs = ["power_sys.h"],
    deps = MODEL_DEPS + [":rotor"],
)

makani_cc_library(
    name = "sea",
    srcs = ["sea.cc"],
    hdrs = ["sea.h"],
    deps = MODEL_DEPS + [":environment"],
)

makani_cc_library(
    name = "simple_power_sys",
    srcs = ["simple_power_sys.cc"],
    hdrs = ["simple_power_sys.h"],
    deps = MODEL_DEPS + [
        ":power_sys",
        ":rotor",
    ],
)

makani_cc_library(
    name = "spring_constraint",
    srcs = ["spring_constraint.cc"],
    hdrs = ["spring_constraint.h"],
    deps = MODEL_DEPS,
)

makani_cc_library(
    name = "stacked_power_sys",
    srcs = ["stacked_power_sys.cc"],
    hdrs = ["stacked_power_sys.h"],
    deps = MODEL_DEPS + [
        ":power_sys",
        ":rotor",
    ],
)

makani_cc_library(
    name = "tether",
    srcs = ["tether.cc"],
    hdrs = ["tether.h"],
    deps = MODEL_DEPS + [
        ":environment",
        ":wing",
    ],
)

makani_cc_library(
    name = "wing",
    srcs = ["rigid_bodies/wing.cc"],
    hdrs = ["rigid_bodies/wing.h"],
    deps = RIGID_BODY_DEPS + [
        ":environment",
        ":servo",
    ],
)

makani_cc_library(
    name = "rotor",
    srcs = ["actuators/rotor.cc"],
    hdrs = ["actuators/rotor.h"],
    data = ["//database:rotor_databases"],
    deps = ACTUATOR_DEPS + [":environment"],
)

makani_cc_library(
    name = "servo",
    srcs = ["actuators/servo.cc"],
    hdrs = ["actuators/servo.h"],
    deps = ACTUATOR_DEPS,
)

makani_cc_library(
    name = "tether_release",
    srcs = ["actuators/tether_release.cc"],
    hdrs = ["actuators/tether_release.h"],
    deps = ACTUATOR_DEPS,
)

makani_cc_library(
    name = "winch",
    srcs = ["actuators/winch.cc"],
    hdrs = ["actuators/winch.h"],
    deps = ACTUATOR_DEPS,
)

makani_cc_library(
    name = "gps",
    srcs = ["sensors/gps.cc"],
    hdrs = ["sensors/gps.h"],
    deps = SENSOR_DEPS + [
        ":delayed_signal",
        ":wing",
    ],
)

makani_cc_library(
    name = "gsg",
    srcs = ["sensors/gsg.cc"],
    hdrs = ["sensors/gsg.h"],
    deps = SENSOR_DEPS + [
        ":measurement",
        ":perch",
        ":tether",
    ],
)

makani_cc_library(
    name = "ground_station_v2_sensors",
    srcs = ["sensors/ground_station_v2_sensors.cc"],
    hdrs = ["sensors/ground_station_v2_sensors.h"],
    deps = SENSOR_DEPS,
)

makani_cc_library(
    name = "gs_gps",
    srcs = ["sensors/gs_gps.cc"],
    hdrs = ["sensors/gs_gps.h"],
    deps = SENSOR_DEPS + [
        ":ground_station_v2",
        "//avionics/common:tether_convert",
    ],
)

makani_cc_library(
    name = "imu",
    srcs = ["sensors/imu.cc"],
    hdrs = ["sensors/imu.h"],
    deps = SENSOR_DEPS + [
        ":environment",
        ":delayed_signal",
        ":imu_mount",
        ":measurement",
        ":wing",
    ],
)

makani_cc_library(
    name = "joystick",
    srcs = ["sensors/joystick.cc"],
    hdrs = ["sensors/joystick.h"],
    deps = SENSOR_DEPS,
)

makani_cc_library(
    name = "loadcell",
    srcs = ["sensors/loadcell.cc"],
    hdrs = ["sensors/loadcell.h"],
    deps = SENSOR_DEPS + [":measurement"],
)

makani_cc_library(
    name = "pitot",
    srcs = ["sensors/pitot.cc"],
    hdrs = ["sensors/pitot.h"],
    deps = SENSOR_DEPS + [
        ":environment",
        ":measurement",
        ":wing",
    ],
)

makani_cc_library(
    name = "rotor_sensor",
    srcs = ["sensors/rotor_sensor.cc"],
    hdrs = ["sensors/rotor_sensor.h"],
    deps = SENSOR_DEPS + [
        ":measurement",
        ":power_sys",
        ":rotor",
    ],
)

makani_cc_library(
    name = "servo_sensor",
    srcs = ["sensors/servo_sensor.cc"],
    hdrs = ["sensors/servo_sensor.h"],
    deps = SENSOR_DEPS + [
        ":measurement",
        ":servo",
    ],
)

makani_cc_library(
    name = "winch_sensor",
    srcs = ["sensors/winch_sensor.cc"],
    hdrs = ["sensors/winch_sensor.h"],
    deps = SENSOR_DEPS + [
        ":perch",
        ":winch",
    ],
)

makani_cc_library(
    name = "wind_sensor",
    srcs = ["sensors/wind_sensor.cc"],
    hdrs = ["sensors/wind_sensor.h"],
    deps = SENSOR_DEPS + [
        ":environment",
        ":measurement",
        ":perch",
    ],
)

makani_cc_library(
    name = "full_system",
    srcs = ["full_system.cc"],
    hdrs = ["full_system.h"],
    deps = [
        ":base_system_model",
        ":buoy",
        ":environment",
        ":gps",
        ":ground_station_v2",
        ":ground_station_v2_sensors",
        ":gs_gps",
        ":gsg",
        ":high_voltage_harness",
        ":imu",
        ":imu_mount",
        ":joystick",
        ":loadcell",
        ":perch",
        ":pitot",
        ":platform",
        ":power_sys",
        ":rotor",
        ":rotor_sensor",
        ":sea",
        ":servo",
        ":servo_sensor",
        ":simple_power_sys",
        ":spring_constraint",
        ":stacked_power_sys",
        ":tether",
        ":tether_release",
        ":winch",
        ":winch_sensor",
        ":wind_sensor",
        ":wing",
        "//sim/physics:ground_frame",
    ],
)

makani_cc_library(
    name = "tether_simulator_system",
    srcs = ["tether_simulator_system.cc"],
    hdrs = ["tether_simulator_system.h"],
    deps = [
        ":base_system_model",
        ":environment",
        ":tether",
        "//lib/hdf5_to_pcap:h5log_reader",
        "//sim/math:interp",
        "//sim/physics:ground_frame",
    ],
)

makani_cc_test(
    name = "ground_station_util_test",
    srcs = ["ground_station_util_test.cc"],
    deps = [
        ":ground_station_util",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "ground_station_v2_test",
    srcs = ["actuators/ground_station_v2_test.cc"],
    deps = [
        ":ground_station_v2",
        "//control:system_params",
    ],
)

makani_cc_test(
    name = "servo_test",
    srcs = ["actuators/servo_test.cc"],
    deps = [":servo"],
)

makani_cc_test(
    name = "pitot_test",
    srcs = ["sensors/pitot_test.cc"],
    deps = [
        ":pitot",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "gs_gps_test",
    srcs = ["sensors/gs_gps_test.cc"],
    deps = [
        ":environment",
        ":gs_gps",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "rotor_sensor_test",
    srcs = ["sensors/rotor_sensor_test.cc"],
    copts = ["-Wno-effc++"],
    data = ["//database/wind:turbsim"],
    deps = [
        ":rotor_sensor",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "wind_sensor_test",
    srcs = ["sensors/wind_sensor_test.cc"],
    deps = [":wind_sensor"],
)

makani_cc_test(
    name = "delayed_signal_test",
    srcs = ["signals/delayed_signal_test.cc"],
    deps = [":delayed_signal"],
)

makani_cc_test(
    name = "measurement_test",
    srcs = ["signals/measurement_test.cc"],
    deps = [
        ":measurement",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "perch_test",
    srcs = ["perch_test.cc"],
    deps = [
        ":perch",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "tether_test",
    srcs = ["tether_test.cc"],
    deps = [
        ":full_system",
        ":tether",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "wing_test",
    timeout = "moderate",
    srcs = ["rigid_bodies/wing_test.cc"],
    data = ["//database/wind:turbsim"],
    deps = [
        ":wing",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "sea_test",
    timeout = "moderate",
    srcs = ["sea_test.cc"],
    deps = [
        ":sea",
        "//lib/util:test_util",
    ],
)

makani_cc_test(
    name = "buoy_test",
    timeout = "moderate",
    srcs = ["rigid_bodies/buoy_test.cc"],
    deps = [
        ":buoy",
        "//lib/util:test_util",
    ],
)
