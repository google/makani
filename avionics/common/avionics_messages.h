/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// This file defines the messages sent over the avionics network.
//
// Notes:
//   Structs of the name FooMessage are messages sent on the network.

#ifndef AVIONICS_COMMON_AVIONICS_MESSAGES_H_
#define AVIONICS_COMMON_AVIONICS_MESSAGES_H_

#include <stdint.h>

#include "avionics/common/actuator_types.h"
#include "avionics/common/ars308_types.h"
#include "avionics/common/batt_types.h"
#include "avionics/common/build_info_types.h"
#include "avionics/common/decawave_types.h"
#include "avionics/common/eop_messages.h"
#include "avionics/common/faa_light_types.h"
#include "avionics/common/faults.h"
#include "avionics/common/gill_types.h"
#include "avionics/common/gps_types.h"
#include "avionics/common/imu_types.h"
#include "avionics/common/loadcell_types.h"
#include "avionics/common/motor_adc_defines.h"
#include "avionics/common/motor_angle_types.h"
#include "avionics/common/motor_foc_types.h"
#include "avionics/common/motor_profiler_types.h"
#include "avionics/common/motor_thermal_types.h"
#include "avionics/common/mvlv_types.h"
#include "avionics/common/network_config.h"
#include "avionics/common/network_diag_types.h"
#include "avionics/common/novatel_types.h"
#include "avionics/common/plc_messages.h"
#include "avionics/common/plc_types.h"
#include "avionics/common/septentrio_types.h"
#include "avionics/common/servo_types.h"
#include "avionics/common/short_stack_types.h"
#include "avionics/common/winch_messages.h"
#include "avionics/firmware/drivers/microhard_types.h"
#include "avionics/firmware/identity/identity_types.h"
#include "avionics/firmware/monitors/aio_types.h"
#include "avionics/firmware/monitors/batt_types.h"
#include "avionics/firmware/monitors/cs_types.h"
#include "avionics/firmware/monitors/fc_types.h"
#include "avionics/firmware/monitors/ground_io_types.h"
#include "avionics/firmware/monitors/joystick_types.h"
#include "avionics/firmware/monitors/loadcell_types.h"
#include "avionics/firmware/monitors/mvlv_types.h"
#include "avionics/firmware/monitors/recorder_types.h"
#include "avionics/firmware/monitors/servo_types.h"
#include "avionics/firmware/monitors/short_stack_types.h"
#include "avionics/firmware/network/net_mon_types.h"
#include "avionics/firmware/params/param_types.h"
#include "avionics/firmware/serial/serial_params.h"
#include "avionics/motor/monitors/types.h"
#include "avionics/network/aio_labels.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/experiments/experiment_types.h"

#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

// Common structures.

typedef struct {
  // Total network received packets = received_valid_aio_packets
  //                                  + invalid_aio_packets
  //                                  + non_aio_packets.
  int16_t received_valid_aio_packets;     // Valid version and length.
  int16_t received_invalid_aio_packets;   // Invalid version or length.
  int16_t received_non_routine_packets;   // Not expected during flight.
  int16_t received_arp_request_packets;   // ARP requests.
  int16_t received_icmp_request_packets;  // ICMP requests.
  int16_t received_probe_packets;         // Probe packets received.
  int16_t sent_aio_packets;               // AIO packets sent.
} AioStats;

typedef struct {
  int16_t unique_packets;   // Total number of CVT updates.
  int16_t unread_packets;   // Messages overwritten before read.
  int16_t invalid_packets;  // Invalid source, type, length, or sequence number.
  uint16_t event_codes;     // Bitmask of CvtEventCodes.
} CvtStats;

// Network status for each node.
typedef struct {
  AioStats aio_stats;
  CvtStats cvt_stats;
} NetworkStatus;

typedef enum {
  kSelfTestFailureIncompatibleHardware,
  kSelfTestFailureInvalidBootloaderConfig,
  kSelfTestFailureInvalidCalibParams,
  kSelfTestFailureInvalidCarrierSerialParams,
  kSelfTestFailureInvalidConfigParams,
  kSelfTestFailureInvalidNetworkIdentity,
  kSelfTestFailureInvalidSerialParams,
} SelfTestFailure;

typedef struct {
  BuildInfo build_info;
  SerialParams serial_params;
  SerialParams carrier_serial_params;
  SelfTestFailure failure;
  char text[128];
} SelfTestMessage;

typedef struct {
  int32_t latency;  // [us]
  int32_t time_of_week;  // [ms]
  uint8_t source;  // AioNode who reported the GPS time.
} GpsTimeData;

// AioNodeStatus.status flags.
typedef enum {
  kAioNodeStatusPowerUpReset    = (1 << 2),
  kAioNodeStatusWatchdogReset   = (1 << 3),
  kAioNodeStatusOscillatorReset = (1 << 4),
  kAioNodeStatusCpuReset        = (1 << 5),
  kAioNodeStatusSoftwareReset   = (1 << 6),
  kAioNodeStatusEsmError        = (1 << 7)
} AioNodeStatusFlag;

// Common status packet for each node.
typedef struct {
  uint16_t status;  // See AioNodeStatusFlag.
} AioNodeStatus;

// TODO: Add parameter and board info.
typedef struct {
  AioNodeStatus node_status;
  AccessSwitchStats switch_stats;
  BuildInfo build_info;
  SerialParams serial_params;
  SerialParams carrier_serial_params;
  NetworkStatus network_status;
  GpsTimeData gps_time;
} SlowStatusMessage;

typedef struct {
  AioNodeStatus node_status;
  BuildInfo build_info;
  bool bootloader_segment;
  SerialParams serial_params;
  HardwareType hardware_type;
  CarrierHardwareType carrier_hardware_type;
} BootloaderSlowStatusMessage;

typedef struct {
  AioNode target;
} DumpRoutesRequestMessage;

typedef struct {
  int16_t index;
  AddressRouteEntry entry;
} DumpRoutesResponseMessage;

// Common AIO network latency measurement packets.
typedef struct {
  int32_t timestamp;
} LatencyProbeMessage;

typedef struct {
  int32_t timestamp;
} LatencyResponseMessage;

#define NUM_DISKS 4
#define FS_PATH_LENGTH 20

typedef enum {
  kDiskInfoMounted    = (1 << 0),
  kDiskInfoWriteable  = (1 << 1),
  kDiskInfoUsageValid = (1 << 2),
} DiskInfoFlag;

typedef struct {
  int64_t total_blocks;
  int64_t available_blocks;
  int64_t total_inodes;
  int64_t available_inodes;
  int32_t block_size;
  int8_t flags;  // See DiskInfoFlag.
  char path[FS_PATH_LENGTH];
} DiskInfo;

typedef struct {
  int32_t uptime;          // In seconds.
  float load_averages[3];  // For the past 1, 5, and 15 minutes.
  // Memory sizes are in multiples of mem_units bytes.
  uint32_t total_memory;
  uint32_t available_memory;
  uint32_t mem_units;
  int16_t num_processes;
  int8_t num_cpus;
} SysInfo;

typedef enum {
  kTemperatureInfoFlagSsdValid      = (1 << 0),
  kTemperatureInfoFlagCpuZone0Valid = (1 << 1),
  kTemperatureInfoFlagCpuZone1Valid = (1 << 2),
} TemperatureInfoFlag;

// All are in degrees Celsius.
typedef struct {
  int16_t ssd;
  int16_t cpu_zone_0;
  int16_t cpu_zone_1;
  int8_t flags;  // See TemperatureInfoFlag.
} TemperatureInfo;

typedef uint8_t GitHash[20];

typedef struct {
  DiskInfo disk_info[NUM_DISKS];
  SysInfo sys_info;
  TemperatureInfo temperature_info;
  GitHash git_hash;  // Git hash of yocto distro.
  BuildInfo build_info;
  uint8_t app_is_running;
} Q7SlowStatusMessage;

typedef enum {
  kControllerBitA = (1 << 0),
  kControllerBitB = (1 << 1),
  kControllerBitC = (1 << 2),
} ControllerBitFlag;

typedef struct {
  uint8_t controllers_used;  // See ControllerBitFlag.
} CommandArbiterStatus;

// Common messages for parameter management.

typedef struct {
  int32_t node_id;  // See AioNode enum.
  int8_t section;  // See ParamsSection.
  uint16_t offset;
} ParamRequestMessage;

typedef struct {
  int8_t section;  // See ParamsSection.
  uint16_t offset;
  uint16_t length;
  uint8_t data[1024];
} ParamResponseMessage;

// LV battery.

typedef struct {
  BattStateCommand state_command;
  uint32_t batt_signal;
} BattCommandMessage;

typedef struct {
  AioModuleMonitorData aio_mon;
  BattMonitorData batt_mon;
} BatteryStatusMessage;

typedef struct {
  float cell_stack_voltage;
  bool uses_direct_charge;
} BattPairedStatusMessage;

// EoP modem.

typedef struct {
  EopModemStatusMessage modem;
} EopSlowStatusMessage;

// Flight computer.

typedef struct {
  uint8_t sequence;
  int32_t flight_mode;
} ControllerSyncMessage;

// Flight controller commands (Q7 output).
typedef struct {
  int16_t motor_command;  // See MotorCommandFlag.
  float motor_speed_upper_limit[kNumMotors];  // [rad/s]
  float motor_speed_lower_limit[kNumMotors];  // [rad/s]
  float motor_torque[kNumMotors];             // [Nm]
  float servo_angle[kNumServos];              // [rad] in interval (-PI, PI).
  double detwist_position;                    // [rad] Multi-turn.
  float winch_velocity;                       // [rad/s]
  float gs_azi_target;                        // [rad]
  float gs_azi_dead_zone;                     // [rad]
  uint8_t gs_mode_request;  // See GroundStationMode.
  uint8_t gs_unpause_transform;
  uint8_t tether_release;
  uint32_t tether_release_safety_code;
} ControllerCommandMessage;

typedef struct {
  uint8_t force_hover_accel;
  uint8_t force_high_tension;
  uint8_t force_reel;
  uint8_t gs_unpause_transform;
  uint8_t force_detwist_turn_once;
  uint32_t safety_code;
  uint8_t experiment_type;
  uint8_t experiment_case_id;
} FlightCommandMessage;

#define MAX_NUM_UWB_NODES 4

typedef struct {
  uint16_t source_node_id;
  uint8_t num_nodes;
  NodeDistance node_distances[MAX_NUM_UWB_NODES];
} DecawaveMessage;

typedef struct {
  int16_t motor_command;          // See MotorCommandFlag.
  float motor_speed_upper_limit[kNumDynoMotors];  // [rad/s]
  float motor_speed_lower_limit[kNumDynoMotors];  // [rad/s]
  float motor_torque[kNumDynoMotors];             // [Nm]
} DynoCommandMessage;

typedef enum {
  kMotorAngleCalModeForceSigned = -1,
  kMotorAngleCalModeNoise,
  kMotorAngleCalModeAngle
} MotorAngleCalMode;

typedef struct {
  int32_t index;
  int32_t mode;  // See MotorAngleCalMode
  float angle;
  int32_t a1;
  int32_t b1;
  int32_t a2;
  int32_t b2;
} MotorCalibrationMessage;

// Flight computer sensors.

// Pitot tube.
typedef struct {
  int32_t latency_usec;
  float speed;
  float speed_temp;
  float altitude;
  float altitude_temp;
  float pitch;
  float pitch_temp;
  float yaw;
  float yaw_temp;
} PitotSensor;

typedef struct {
  uint16_t status;  // See Adis16488StatusFlag.
  uint16_t error;   // See Adis16488ErrorFlag.
  ImuRawData raw;
} FlightComputerImuMessage;

typedef enum {
  kFlightComputerFlagNoGps             = 1 << 0,
  kFlightComputerFlagPitotAltitudeDiag = 1 << 1,
  kFlightComputerFlagPitotPitchDiag    = 1 << 2,
  kFlightComputerFlagPitotSpeedDiag    = 1 << 3,
  kFlightComputerFlagPitotYawDiag      = 1 << 4,
} FlightComputerFlag;

typedef enum {
  kFlightComputerWarningGps           = 1 << 0,
  kFlightComputerWarningImu           = 1 << 1,
  kFlightComputerWarningImuData       = 1 << 2,
  kFlightComputerWarningPitotAltitude = 1 << 3,
  kFlightComputerWarningPitotPitch    = 1 << 4,
  kFlightComputerWarningPitotSpeed    = 1 << 5,
  kFlightComputerWarningPitotYaw      = 1 << 6,
  kFlightComputerWarningFpvEnabled    = 1 << 7,  // FPV interferes with GPS-L2.
} FlightComputerWarning;

typedef struct {
  // Integrity monitoring.
  StatusFlags flags;
  AioModuleMonitorData aio_mon;
  FcMonitorData fc_mon;

  // Sensor data.
  ImuConingScullingData cs[3];  // Shift register; index 0 contains the latest.
  ImuAuxSensorData aux;
  PitotSensor pitot;
  uint8_t pitot_cover_status;     // See PitotCoverStatus.
  int32_t pps_latency_usec;
} FlightComputerSensorMessage;

#define GPS_EPHEMERIDES_MAX 10

// Gps satellites message.
typedef struct {
  int32_t latency_usec;
  GpsIonosphere iono;
  GpsUtc utc;
  GpsEphemeris eph[GPS_EPHEMERIDES_MAX];
} GpsSatellitesMessage;

// Gps timestamp.
typedef struct {
  int32_t latency;  // [us]
  int32_t time_of_week;  // [ms]
} GpsTimeMessage;

// NovAtel compass.
typedef struct {
  int32_t heading_latency;
  NovAtelLogHeading heading;
  int32_t heading_rate_latency;
  NovAtelLogHeadingRate heading_rate;
} NovAtelCompassMessage;

// NovAtel solution.
typedef struct {
  int32_t best_xyz_latency;
  NovAtelLogBestXyz best_xyz;
  NovAtelLogRxStatus rx_status;
  float avg_cn0;
  float max_cn0;
  uint8_t idle_time;
} NovAtelSolutionMessage;

// NovAtel observations.
typedef struct {
  int32_t pps_latency_usec;
  int32_t latency_usec;
  NovAtelLogRange range;
} NovAtelObservationsMessage;

// Septentrio solution.
typedef struct {
  int32_t latency_usec;
  SeptentrioBlockPvtCartesian pvt_cartesian;
  SeptentrioBlockPosCovCartesian pos_cov_cartesian;
  SeptentrioBlockVelCovCartesian vel_cov_cartesian;
  SeptentrioBlockBaseVectorCart base_vector_cart;
  float avg_cn0;
  float max_cn0;
} SeptentrioSolutionMessage;

// Septentrio observations.
typedef struct {
  int32_t pps_latency_usec;
  int32_t latency_usec;
  SeptentrioBlockMeasEpoch meas_epoch;
} SeptentrioObservationsMessage;

typedef struct {
  AioModuleMonitorData aio_mon;
  FcMonitorData fc_mon;
} GpsStatusMessage;

// FAA light messages.

// FAA light status.
typedef struct {
  LightInputParams input_params[kNumLightTypes];
  LightTiming light_timing;
} FaaLightStatusMessage;

// FAA light acknowledge flash parameters.
typedef struct {
  uint8_t id;
  float value;
} FaaLightAckParamMessage;

// FAA light set flash parameters.
typedef struct {
  AioNode target;
  uint8_t id;
  float value;
} FaaLightSetParamMessage;

// FAA light get flash parameters.
typedef struct {
  AioNode target;
  uint8_t id;
} FaaLightGetParamMessage;

// First person video (FPV) on/off.
typedef struct {
  uint8_t enable;
  uint32_t safety_code;
} FpvSetStateMessage;

// Motor status.
typedef struct {
  uint16_t motor_status;  // See MotorStatusFlag.
  uint32_t motor_error;  // See MotorErrorFlag.
  uint32_t motor_warning;  // See MotorWarningFlag.
  ActuatorState state;  // Derived from motor_status.
  float bus_current;
  float bus_voltage;
  float chassis_voltage;
  float cm_voltage;
  float omega;
  float omega_upper_limit;
  float omega_lower_limit;
  float torque_cmd;
  float id;
  float id_cmd;
  float iq;
  float iq_cmd;
  float vd;
  float vq;
  float current_correction;
  float speed_correction;
  float voltage_pair_bias;
  float v_supply_primary;
  float v_supply_auxiliary;
  float temps[kNumMotorThermalChannels];  // See MotorThermalChannel.
  ProfilerOutput profiler_output;
  MotorMonitorData motor_mon;
  CommandArbiterStatus cmd_arbiter;
} MotorStatusMessage;

// High rate message used for power balancing feedback.
typedef struct {
  uint16_t motor_status;
  uint32_t motor_error;
  float bus_current;
  float bus_voltage;
  float current_correction;
  float iq_cmd_residual;
} MotorStackingMessage;

typedef struct {
  ActuatorStateCommand command;
  uint32_t command_data;
  uint8_t selected_motors;
} MotorSetStateMessage;

typedef MotorSetStateMessage DynoMotorSetStateMessage;

// High rate debug messages for monitoring motor behavior.
typedef struct {
  uint16_t motor_status;
  uint32_t motor_error;
  uint32_t motor_warning;
  float bus_current;
  float bus_voltage;
  float chassis_voltage;
  float cm_voltage;
  float theta;
  float omega;
  float omega_upper_limit;
  float omega_lower_limit;
  float torque_cmd;
  float iq_upper_limit;
  float iq_lower_limit;
  float iq_cmd_residual;
  float kt_scale;
  float id;
  float id_cmd;
  float iq;
  float iq_cmd;
  float vd;
  float vq;
  float current_correction;
  float speed_correction;
  float voltage_pair_bias;
  float voltage_stack_mean;
  SensorProfileDiag angle_sensor;
  uint16_t sequence[kNumMotors];
} MotorDebugMessage;

// High rate message.
#define MOTOR_ISR_DIAG_MESSAGE_LENGTH 16
typedef struct {
  uint32_t total;  // Count of the total number of samples sent starting at 1.
  uint32_t num_samples;
  uint32_t errors[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
  uint32_t warnings[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
  float vbus[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
  float ibus[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
  float ia[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
  float ib[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
  float ic[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
  float sin[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
  float cos[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
  float vab_ref[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
  float vab_angle[MOTOR_ISR_DIAG_MESSAGE_LENGTH];
} MotorIsrDiagMessage;

typedef struct {
  uint16_t v_in_monitor[NUM_ADC_SAMPLES];
  uint16_t chassis_voltage[NUM_ADC_SAMPLES];
  uint16_t phase_a_current[NUM_ADC_SAMPLES];
  uint16_t phase_b_current[NUM_ADC_SAMPLES];
  uint16_t bus_current[NUM_ADC_SAMPLES];
  uint16_t bus_voltage[NUM_ADC_SAMPLES];
  uint16_t cm_voltage[NUM_ADC_SAMPLES];
  uint16_t phase_c_current[NUM_ADC_SAMPLES];
  uint16_t phase_b_aux_current[NUM_ADC_SAMPLES];
  uint16_t v_aux_monitor[NUM_ADC_SAMPLES];
} MotorAdcLogMessage;

typedef struct {
  int32_t timestep;

  // Motor State.
  float motor_state_ia;
  float motor_state_ib;
  float motor_state_ic;
  float motor_state_v_bus;
  float motor_state_i_bus;
  float motor_state_theta_elec;  // Electrical angle.
  float motor_state_omega_mech;  // Mechanical angular rate.

  // FOC State.
  float foc_state_id_int;
  float foc_state_iq_int;
  float foc_state_id_error;
  float foc_state_iq_error;
  float foc_state_omega_int;
  float foc_state_omega_error_last;

  // FOC Current (actual).
  float foc_current_actual_id;
  float foc_current_actual_iq;
  float foc_current_actual_i0;

  // FOC Current (command).
  float foc_current_desired_id;
  float foc_current_desired_iq;
  float foc_current_desired_i0;

  // FOC Voltage.
  float foc_voltage_vd;
  float foc_voltage_vq;
  float foc_voltage_v_ref;
  float foc_voltage_angle;

  // Commanded Values.
  float torque_cmd;
  float omega_upper_limit;
  float omega_lower_limit;

  // Motor Errors.
  uint32_t errors;
  uint32_t warnings;
} MotorIsrLogMessage;

typedef struct {
  uint8_t selected_motors;
  uint8_t id;
  float value;
} MotorSetParamMessage;

typedef MotorSetParamMessage DynoMotorSetParamMessage;

typedef struct {
  uint8_t selected_motors;
  uint8_t id;
} MotorGetParamMessage;

typedef MotorGetParamMessage DynoMotorGetParamMessage;

typedef struct {
  uint8_t id;
  float value;
} MotorAckParamMessage;

// MVLV message
typedef struct {
  MvlvStateCommand state_command;
  uint32_t mvlv_signal;
} MvlvCommandMessage;

typedef struct {
  AioModuleMonitorData aio_mon;
  MvlvMonitorData mvlv_mon;
} MvlvStatusMessage;

// Pitot cover open/closed.
typedef struct {
  uint8_t cover;
  uint32_t safety_code;
} PitotSetStateMessage;

typedef struct {
  AioModuleMonitorData aio_mon;
  RecorderMonitorData recorder_mon;
} RecorderStatusMessage;

typedef struct {
  ActuatorStateCommand state_command;
  uint32_t servo_arming_signal;
  uint16_t selected_servos;
} ServoSetStateMessage;

typedef struct {
  uint16_t selected_servos;
  int16_t param;
  uint32_t value;
} ServoSetParamMessage;

typedef struct {
  uint16_t selected_servos;
  int16_t param;
} ServoGetParamMessage;

typedef struct {
  int16_t param;
  uint32_t value;
} ServoAckParamMessage;

typedef struct {
  uint8_t event;
  int32_t seconds;
  uint32_t error_bits;
} ServoErrorLogEntry;

#define SERVO_ERROR_LOG_ENTRIES 10
typedef struct {
  ServoErrorLogEntry data[SERVO_ERROR_LOG_ENTRIES];
} ServoErrorLogMessage;

typedef struct {
  uint16_t selected_servos;
} ServoClearErrorLogMessage;

// Copley R22 servo drive status.
typedef struct {
  int32_t angle;
  int32_t angular_velocity;
  int32_t current;
  int32_t current_limit;
  uint32_t status_bits;
  int16_t temperature;
} R22Status;

// Complete servo status.
typedef struct {
  R22Status r22;
  StatusFlags flags;
  ActuatorState state;
  AioModuleMonitorData aio_mon;
  ServoMonitorData servo_mon;
  float angle_desired;
  float angle_measured;
  float angle_estimate;
  float angle_variance;
  float angle_bias;
  float angle_feedback;
  float angular_velocity;
  CommandArbiterStatus cmd_arbiter;
} ServoStatusMessage;

typedef ServoStatusMessage ServoDebugMessage;

// Shared state for paired servos.  This message is intended to be sent as
// one of the Elevator/Rudder versions below.
typedef struct {
  ServoInputState input;
  ServoControlState control_state;
  ActuatorState state;
  int32_t latency_usec;
} ServoPairedStatusMessage;

typedef ServoPairedStatusMessage ServoPairedStatusRudderMessage;
typedef ServoPairedStatusMessage ServoPairedStatusElevatorMessage;

typedef struct {
  AioModuleMonitorData aio_mon;
  LoadcellMonitorData loadcell_mon;
  BridleJuncData bridle_junc;
  LoadcellCommand command;
  LoadcellData loadcell_data;
  float angle_alpha;
  float angle_beta;
  ActuatorState tether_release_state;
  bool tether_release_fully_armed;
  bool tether_released;
  uint32_t tether_released_safety_code;
  StatusFlags status;
} LoadcellMessage;

// Send command to all loadcells attached to servo board.
typedef struct {
  int16_t selected;  // Bitmask of servo boards.
  LoadcellCommand command;
} LoadcellCommandMessage;

typedef struct {
  ActuatorStateCommand state_command;
  uint32_t arming_signal;
  uint8_t selected_loadcells;
} TetherReleaseSetStateMessage;

// Short Stack operator command message.
typedef struct {
  ShortStackCommandValue command_value;
  uint32_t command_signal;
} ShortStackCommandMessage;

// Short stack message to motors for determining fault recovery.
typedef struct {
  uint16_t firing_status;
  float motor_voltage[kNumMotors / 2];
} ShortStackStackingMessage;

// Short Stack AIO node status message.
typedef struct {
  AioModuleMonitorData aio_mon;
  ShortStackMonitorData short_stack_mon;
} ShortStackStatusMessage;

typedef enum {
  kPortErrorOk               = 0,
  kRxJabberPacket            = (1 << 0),
  kRxAlignmentError          = (1 << 1),
  kRxFrameCountSequenceError = (1 << 2),
  kRxFragmentError           = (1 << 3),
  kRxSymbolError             = (1 << 4),
  kRxInRangeError            = (1 << 5),
  kRxOutOfRangeError         = (1 << 6),
  kTxCongestionDrops         = (1 << 7)
} PortErrorFlag;

// TODO: Add parameter and board info.
typedef struct {
  AioNodeStatus node_status;
  CoreSwitchStats switch_stats;
  BuildInfo build_info;
  SerialParams serial_params;
  NetworkStatus network_status;
  GpsTimeData gps_time;
} CoreSwitchSlowStatusMessage;

typedef struct {
  CsMonitorData cs_mon;
  uint32_t disabled_port_mask;
  MicrohardStatus microhard_status;
} CoreSwitchStatusMessage;

typedef struct {
  AioNode target;
  uint32_t disable_port_mask;
} CoreSwitchConnectionSelectMessage;

typedef enum {
  kJoystickWarningNotPresent = (1 << 0),
} JoystickWarning;

// Joystick commands.
typedef struct {
  StatusFlags status;  // See JoystickWarning.

  float roll;      // -1.0 to 1.0.
  float pitch;     // -1.0 to 1.0.
  float yaw;       // -1.0 to 1.0.
  float throttle;  //  0.0 to 1.0.

  // Values of switches correspond to JoystickSwitchPositionLabel.
  uint8_t tri_switch;
  uint8_t momentary_switch;

  uint32_t tether_release_interlock_code;
  uint32_t scuttle_code;
} JoystickStatusMessage;

typedef struct {
  AioModuleMonitorData aio_mon;
  JoystickMonitorData joystick_mon;
  GroundIoMonitorData ground_io_mon;
  MicrohardStatus microhard_status;
} JoystickMonitorStatusMessage;

typedef struct {
  bool enable_raw;
} JoystickCommandMessage;

#define JOYSTICK_NUM_RAW_CHANNELS 7
typedef struct {
  uint32_t channel[JOYSTICK_NUM_RAW_CHANNELS];
} JoystickRawStatusMessage;

// Ground sensors.

typedef enum {
  kGsDrumEncodersWarningGsgAzimuth    = 1 << 0,
  kGsDrumEncodersWarningGsgElevation  = 1 << 1,
  kGsDrumEncodersWarningDetwist       = 1 << 2,
} GsDrumEncodersWarning;

typedef enum {
  kGsDrumEncodersErrorGsgAzimuth    = 1 << 0,
  kGsDrumEncodersErrorGsgElevation  = 1 << 1,
  kGsDrumEncodersErrorDetwist       = 1 << 2,
} GsDrumEncodersError;

typedef struct {
  StatusFlags status;  // See GsDrumEncodersWarning.
  float gsg_azi;
  float gsg_ele;
  float detwist;
} GsDrumEncoders;

typedef enum {
  kGsPerchEncodersWarningPerchAzimuth       = 1 << 0,
  kGsPerchEncodersWarningLevelwindElevation = 1 << 1,
  kGsPerchEncodersWarningDrumPosition       = 1 << 2,
  kGsPerchEncodersWarningLevelwindShoulder  = 1 << 3,
  kGsPerchEncodersWarningLevelwindWrist     = 1 << 4,
} GsPerchEncodersWarning;

typedef enum {
  kGsPerchEncodersErrorPerchAzimuth       = 1 << 0,
  kGsPerchEncodersErrorLevelwindElevation = 1 << 1,
  kGsPerchEncodersErrorDrumPosition       = 1 << 2,
  kGsPerchEncodersErrorLevelwindShoulder  = 1 << 3,
  kGsPerchEncodersErrorLevelwindWrist     = 1 << 4,
} GsPerchEncodersError;

typedef struct {
  StatusFlags status;        // See GsPerchEncodersWarning.
  float perch_azi;           // AMO4306 single-turn.
  uint8_t perch_azi_flags;   // See Amo4306Flag.
  float levelwind_shoulder;  // RHA507 single-turn.
  float levelwind_wrist;     // RHA507 single-turn.
  float levelwind_ele;       // Sum of shoulder and wrist angles.
  float drum_pos;
} GsPerchEncoders;

typedef struct {
  ActuatorStateCommand state_command;
  uint32_t arming_signal;
} GroundStationDetwistSetStateMessage;

typedef struct {
  AioModuleMonitorData aio_mon;
} GroundStationPlcMonitorStatusMessage;

typedef struct {
  PlcCommandMessage command;
} GroundStationPlcOperatorMessage;

typedef struct {
  ActuatorStateCommand state_command;
  uint32_t arming_signal;
  uint8_t actuator_mask;  // See GroundStationActuator.
} GroundStationSetStateMessage;

typedef struct {
  ActuatorStateCommand state_command;
  uint32_t arming_signal;
} GroundStationWinchSetStateMessage;

typedef struct {
  PlcWinchStatusMessage plc;  // Message forwarded from PLC.
} GroundStationWinchStatusMessage;

typedef struct {
  PlcStatusMessage plc;
  ActuatorState detwist_state;
} GroundStationPlcStatusMessage;

typedef struct {
  PlcGs02ControlInput input;
  PlcGs02ControlOutput output;
} GroundStationControlMessage;

typedef struct {
  GroundStationStatus status;
  ActuatorState actuator_state[kNumGroundStationActuators];
} GroundStationStatusMessage;

typedef struct {
  GsDrumEncoders encoders;
} DrumSensorsMessage;

typedef struct {
  AioModuleMonitorData aio_mon;
  GroundIoMonitorData ground_io_mon;
} DrumSensorsMonitorMessage;

typedef struct {
  float pressure;     // millibars
  float humidity;     // percent
  float dewpoint;     // degrees Celsius
  float temperature;  // degrees Celsius
  uint8_t status;     // See MetPak documentation for error codes (0 = valid).
} GsWeatherData;

typedef struct {
  GsWeatherData weather;
  int32_t weather_latency;      // usec
  GillDataWindmasterUvw wind;
  int32_t wind_latency;         // usec
} GroundStationWeatherMessage;

typedef struct {
  GsPerchEncoders encoders;
} PlatformSensorsMessage;

typedef struct {
  AioModuleMonitorData aio_mon;
  GroundIoMonitorData ground_io_mon;
} PlatformSensorsMonitorMessage;

// RTCM messages sent from ground station GPS to wing GPS for RTK solution.
// RTCM v3 message size = preamble (3 bytes)
//                        + data (0 - 1023 bytes)
//                        + crc24 (3 bytes)
#define GPS_RTCM_PREAMBLE_SIZE 3
#define GPS_RTCM_CRC_SIZE 3
#define GPS_RTCM_DATA_SIZE(message_size)                        \
  (GPS_RTCM_PREAMBLE_SIZE + (message_size) + GPS_RTCM_CRC_SIZE)
#define GPS_RTCM_MAX_DATA_SIZE GPS_RTCM_DATA_SIZE(1023)

typedef struct {
  uint16_t message_number;
  uint8_t data[GPS_RTCM_MAX_DATA_SIZE];
  int16_t length;
} GpsRtcmMessage;

// RTCM1006: Stationary RTK Reference Station ARP with Antenna Height.
typedef struct {
  uint8_t data[GPS_RTCM_DATA_SIZE(21)];
  int16_t length;
} GpsRtcm1006Message;

// RTCM1033: Receiver and Antenna Descriptors.
typedef struct {
  uint8_t data[GPS_RTCM_DATA_SIZE(170)];
  int16_t length;
} GpsRtcm1033Message;

// RTCM1072/RTCM1082: MSM2 Compact GNSS PhaseRanges.
typedef struct {
  uint8_t data[GPS_RTCM_DATA_SIZE(258)];
  int16_t length;
} GpsRtcm1072Message;

typedef GpsRtcm1072Message GpsRtcm1082Message;

// RTCM1074/1084: MSM4 Full GNSS PseudoRanges and PhaseRanges plus CNR.
typedef struct {
  uint8_t data[GPS_RTCM_DATA_SIZE(442)];
  int16_t length;
} GpsRtcm1074Message;

typedef GpsRtcm1074Message GpsRtcm1084Message;

// RTCM1230: GLONASS L1 & L2 code phase biases.
typedef struct {
  uint8_t data[GPS_RTCM_DATA_SIZE(96)];
  int16_t length;
} GpsRtcm1230Message;

typedef struct {
  int8_t port;
  int16_t length;
  // Maximum length assumes 100% utilization at 460,800 bps with 8N1.
  uint8_t data[461];
} SerialDebugMessage;

// Ground commands.

// Command to wing.
typedef struct {
  uint16_t command;
  uint16_t id;
  float value[8];
} WingCommandMessage;

// Ground estimate.

typedef struct {
  // Time in seconds (on the GsEstimator) at which the estimates were
  // calculated.
  double time;

  // Attitude of the platform with respect to the ground frame.
  Mat3 dcm_g2p;
  // Rotation rates of the platform with respect to the ground frame.
  Vec3 pqr;
  // Validity flag on attitude and body rates.
  bool attitude_valid;

  // Position and velocity estimates of the platform, with respect to the ground
  // frame.  Note that since the origin of the platform frame and the vessel
  // frame are the same, these apply to both.
  Vec3 Xg, Vg;
  // Validity flag on position and velocity.
  bool position_valid;
} GroundEstimateMessage;

typedef GroundEstimateMessage GroundEstimateSimMessage;

// Ground power. Battery bank and grid diagnostics go here.

// Inverter status.
typedef struct {
  bool modbus_status;
  uint16_t stale_count;
  uint16_t inverter_status;  // See InverterStatusFlag
  uint8_t id;
  float v_dc;
  float i_dc;
  float v_avg_grid;
  float i_avg_grid;
  float p_mean_active_dc;
  float p_mean_active_ac;
  float p_mean_reactive_ac;

  // Common mode measurements.
  float mean_common_mode_v;
  float inst_common_mode_v;

  // Temperatures
  float cb_air_temp;
  float inverter_air_temp;
  float transformer_temp;
  float heatsink1_temp1;
  float heatsink1_temp2;

  // Fault vectors
  uint16_t fault_word1;
  uint16_t fault_word2;
  uint16_t fault_word3;
  uint16_t fault_word4;
  uint16_t fault_word5;
  uint16_t fault_word6;
  uint16_t fault_word7;
  uint16_t fault_word8;

  // Fault Inductor
  uint16_t fault_inductor_status;

  // Tether quantities.
  float tether_compensation_calc;

  // Diagnostics.
  float grid_v_ab;
  float grid_v_bc;
  float grid_v_ca;
  float grid_i_a;
  float grid_i_b;
  float grid_i_c;
  int16_t l1_i_a;
  int16_t l1_i_b;
  int16_t l1_i_c;
} GroundPowerStatusMessage;

// GroundPowerCommandMessage, GroundPowerSetParamMessage, and
// GroundPowerGetParamMessage are all processed through the ground power message
// queue.
typedef struct {
  uint8_t id;
  uint16_t command;  // See GroundPowerCommandFlag.
} GroundPowerCommandMessage;

typedef struct {
  uint8_t id;
  uint16_t modbus_register;
  uint16_t value;
} GroundPowerSetParamMessage;

typedef struct {
  uint8_t id;
  uint16_t modbus_register;
} GroundPowerGetParamMessage;

typedef struct {
  uint8_t id;
  uint16_t modbus_register;
  uint16_t value;
} GroundPowerAckParamMessage;

typedef struct {
  int32_t desired_load_kw;  // Desired load in kW.
} LoadbankSetLoadMessage;

typedef struct {
  bool activate_loadbank;  // On = 1, off = 0.
} LoadbankSetStateMessage;

typedef struct {
  int32_t value;
} LoadbankAckParamMessage;

typedef struct {
  bool value;
} LoadbankStateAckParamMessage;

typedef struct {
  bool loadbank_activated;      // Loadbank on/off bool.
  bool loadbank_power_modbus_status;
  bool kite_power_modbus_status;
  uint32_t loadbank_cmd_modbus_status;
  float loadbank_power_kw;      // Kilowatts.
  float loadbank_kvar;          // KiloVARs.
  float loadbank_kva;           // KiloVAs.
  float kite_power_kw;          // Kilowatts.
  float kite_kvar;              // KiloVARs.
  float kite_kva;               // KiloVAs.
  int32_t desired_net_load_kw;  // Kilowatts.
  int32_t n_requested_relays;   // Each relay controls 100 kW.
  uint16_t relay_mask;          // 1 = active for a given relay.
} LoadbankStatusMessage;

// Logger messages.
#define SYSTEM_NAME_LENGTH 16
#define FLIGHT_NAME_LENGTH 32
typedef struct {
  uint8_t command;  // See LoggerCommandType.
  char system_name[SYSTEM_NAME_LENGTH];
  char flight_name[FLIGHT_NAME_LENGTH];
} LoggerCommandMessage;

typedef struct {
  int32_t elapsed_time;
  int64_t disk_space;
  uint8_t logger_state;  // See LoggerState.
} LoggerStatusMessage;

// Tether messages.
typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  int32_t time_of_week;  // [ms]
} TetherGpsTime;

typedef enum {
  kTetherJoystickFlagFault = 1 << 0,
} TetherJoystickFlag;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;   // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t flags;       // See TetherJoystickFlag.
  float roll;          // -1.0 to 1.0.
  float pitch;         // -1.0 to 1.0.
  float yaw;           // -1.0 to 1.0.
  float throttle;      //  0.0 to 1.0.
  uint8_t tri_switch;  // See JoystickSwitchPositionLabel.
  uint8_t momentary_switch;  // See JoystickSwitchPositionLabel.
  uint32_t tether_release_interlock_code;
  uint32_t scuttle_code;
} TetherJoystick;

typedef enum {
  kTetherPlatformFlagPerchAzimuthFault       = 1 << 0,
  kTetherPlatformFlagLevelwindElevationFault = 1 << 1,
} TetherPlatformFlag;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;    // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t flags;        // See TetherPlatformFlag.
  float perch_azi;      // [rad]
  float levelwind_ele;  // [rad]
} TetherPlatform;

typedef enum {
  kTetherDrumFlagGsgAxis1Fault = 1 << 0,
  kTetherDrumFlagGsgAxis2Fault = 1 << 1,
} TetherDrumFlag;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;  // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t flags;      // See TetherDrumFlag.
  float gsg_axis1;    // [rad] GSG azimuth angle.
  float gsg_axis2;    // [rad] GSG elevation angle.
} TetherDrum;

typedef enum {
  kTetherGroundStationFlagError          = 1 << 0,
  kTetherGroundStationFlagDetwistError   = 1 << 1,
  kTetherGroundStationFlagDrumError      = 1 << 2,
} TetherGroundStationFlag;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;  // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t flags;      // See TetherGroundStationFlag.
  uint8_t mode;       // See GroundStationMode.
  uint8_t transform_stage;
  float drum_angle;
  float detwist_angle;
  uint8_t proximity;
  uint8_t tether_engaged;
} TetherGroundStation;

typedef enum {
  kTetherPlcFlagPlcWarning     = 1 << 0,
  kTetherPlcFlagPlcError       = 1 << 1,
  kTetherPlcFlagDetwistFault   = 1 << 2,
  kTetherPlcFlagDrumFault      = 1 << 3,
  kTetherPlcFlagProximityFault = 1 << 4,
} TetherPlcFlag;

typedef enum {
  kTetherPlcProximityEarlyA = 1 << 0,
  kTetherPlcProximityEarlyB = 1 << 1,
  kTetherPlcProximityFinalA = 1 << 2,
  kTetherPlcProximityFinalB = 1 << 3,
} TetherPlcProximity;

// Specify number of revolutions to wrap the multiturn detwist angle/command.
#define TETHER_DETWIST_REVS 1.0f
#define TETHER_DETWIST_BITS 12

// Specify number of revolutions to wrap the multiturn drum angle/command.
#define TETHER_DRUM_REVS 100.0f
#define TETHER_DRUM_BITS 18

// TODO: Remove proximity flag.
typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;    // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t flags;        // See TetherPlcFlag.
  uint8_t proximity;    // See TetherPlcProximity.
  float detwist_angle;  // [0, 2*PI*TETHER_DETWIST_REVS) [rad].
  float drum_angle;     // [-2*PI*TETHER_DRUM_REVS, 0] [rad].
} TetherPlc;

typedef enum {
  kTetherGpsSolutionStatusNone,
  kTetherGpsSolutionStatusFixedPos,
  kTetherGpsSolutionStatusSingle,
  kTetherGpsSolutionStatusSbasAided,
  kTetherGpsSolutionStatusDifferential,
  kTetherGpsSolutionStatusRtkFloat,
  kTetherGpsSolutionStatusRtkFixed
} TetherGpsSolutionStatus;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;   // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t status;      // See TetherGpsSolutionStatus.
  uint8_t satellites;  // [#]
  float pos_sigma;     // [m]
  int8_t avg_cn0;      // [dB-Hz]
} TetherGpsStatus;

typedef enum {
  kTetherGsGpsPositionFlagFault = 1 << 0,
} TetherGsGpsPositionFlag;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;  // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t flags;      // See TetherGsGpsPositionFlag.
  double ecef[3];     // [m]
} TetherGsGpsPosition;

typedef enum {
  kTetherGsGpsCompassFlagFault = 1 << 0,
} TetherGsGpsCompassFlag;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;    // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t flags;        // See TetherGsGpsPitchFlag.
  float heading;        // [rad]
  float heading_sigma;  // [rad]
  float heading_rate;   // [rad/s]
} TetherGsGpsCompass;

typedef enum {
  kTetherWindStatusGood,
  kTetherWindStatusWarning,
  kTetherWindStatusFault
} TetherWindStatus;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;  // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t status;     // See TetherWindStatus.
  float velocity[3];  // [m/s]
} TetherWind;

typedef enum {
  kTetherWeatherFlagFault = 1 << 0,
} TetherWeatherFlag;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;  // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t flags;      // See TetherWeatherFlag.
  float pressure_pa;  // [Pa]
  float temperature;  // [C]
  float humidity;     // [%]
} TetherWeather;

// The GS azimuth dead zone encoding should near-exactly encode values of
// interest. Currently, we set the resolution to 0.05 deg (but expressed in rad)
// to support both increments 0.1 deg and 0.25 deg.
#define GS_AZI_DEAD_ZONE_BITS 8
#define GS_AZI_DEAD_ZONE_RESOLUTION 8.72664625e-4f
#define GS_AZI_DEAD_ZONE_MAX (float)((1 << GS_AZI_DEAD_ZONE_BITS) - 1)  \
      * GS_AZI_DEAD_ZONE_RESOLUTION

typedef struct {
  int32_t no_update_count;     // Number of frame indices without update.
  uint16_t sequence;           // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t controller_label;    // Arbitrated controller.
  float detwist_angle;         // See TETHER_DETWIST_REVS [rad].
  float winch_velocity;        // [rad/s]
  float gs_azi_target;         // [rad]
  float gs_azi_dead_zone;      // [rad]
  uint8_t gs_mode_request;     // See GroundStationMode.
  uint8_t gs_unpause_transform;
} TetherControlCommand;

typedef enum {
  kTetherControlTelemetryFlagAutoGlideActive = 1 << 0,
  kTetherControlTelemetryFlagReleaseLatched  = 1 << 1,
} TetherControlTelemetryFlag;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint16_t sequence;         // See TETHER_SEQUENCE_ROLLOVER for modulus.
  uint8_t controller_label;  // Arbitrated controller.
  uint8_t flags;             // See TetherControlTelemetryFlag.

  // Rotate through all subsystems with a fault indication.
  uint8_t subsystem;         // See SubsystemLabel.
  uint8_t subsystem_faults;  // See bitmask for corresponding subsystem.

  // Current flight mode and gates for the next flight mode.
  uint8_t flight_mode;         // See FlightMode enum.
  uint16_t flight_mode_gates;  // See bitmask for corresponding mode.
  uint8_t experiment_test_id;
  uint8_t experiment_test_case_id;

  // Experiment configs.
  uint8_t experiment_type;
  uint8_t experiment_case_id;

  // Timers.
  uint32_t flight_mode_time;  // [0.1 s]
  float loop_time;            // [s]

  // Pilot data.
  uint32_t loop_count;  // Rolls over at 2^20 [#].
  float loop_angle;  // [rad]
  float airspeed;    // [m/s]
  float alpha;       // [rad]
  float beta;        // [rad]
  float roll;        // [rad]
  float pitch;       // [rad]
  float yaw;         // [rad]
  float pqr[3];      // [rad/s]
  float pos_g[3];    // [m]
  float vel_g[3];    // [m/s]

  // Crosswind status.
  float target_pos_cw[2];    // [m]
  float current_pos_cw[2];   // [m]
  float delta_aileron;       // [rad]
  float delta_elevator;      // [rad]
  float delta_rudder;        // [rad]
  float delta_inboard_flap;  // [rad]

  // Hover dynamics.
  float tension;          // [N]
  float tension_command;  // [N]
  float thrust;           // [N]
  float thrust_avail;     // [N]
  float moment[3];        // [Nm]
  float moment_avail[3];  // [Nm]
  float gain_ramp_scale;  // [%]
  uint8_t force_detwist_turn_once;
} TetherControlTelemetry;

// The minimal telemetry packets sent over the XLR radio are typically
// composed by the core switches.  However, the control telemetry is
// too large to be processed on the TMS570.  As a workaround, we send
// a dedicated message containing this data from the controller.
typedef TetherControlTelemetry SmallControlTelemetryMessage;

typedef enum {
  kTetherFlightComputerFlagImuGood    = 1 << 0,
  kTetherFlightComputerFlagGpsGood    = 1 << 1,
  kTetherFlightComputerFlagPitotGood  = 1 << 2,
  kTetherFlightComputerFlagFpvEnabled = 1 << 3,
} TetherFlightComputerFlag;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint8_t flags;
} TetherFlightComputer;

typedef enum {
  kTetherMotorForceSigned = -1,
  kTetherMotorTempNacelleAir,
  kTetherMotorTempRotor,
  kTetherMotorTempStatorCoil,
  kTetherMotorTempStatorCore,
  kNumTetherMotorTemps
} TetherMotorTemp;

typedef enum {
  kTetherMotorControllerForceSigned = -1,
  kTetherMotorControllerTempAir,
  kTetherMotorControllerTempBoard,
  kTetherMotorControllerTempCapacitor,
  kTetherMotorControllerTempHeatPlate,
  kNumTetherMotorControllerTemps
} TetherMotorControllerTemp;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint8_t state;  // See ActuatorState enum.
  uint8_t warning;
  uint8_t error;
  int16_t speed;
  int16_t iq;
  int16_t id;
  int16_t bus_voltage;
  int16_t bus_current;
  int16_t motor_temps[kNumTetherMotorTemps];
  int16_t controller_temps[kNumTetherMotorControllerTemps];
} TetherMotorStatus;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint8_t state;     // See ActuatorState enum.
  int16_t r22_temp;  // [C]
  float angle;       // [rad]
} TetherServoStatus;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint8_t state;  // See ActuatorState enum.
  uint8_t interlock_switched;
  uint8_t released;
} TetherReleaseStatus;

typedef enum {
  kTetherBatteryTempBattery1,
  kTetherBatteryTempBattery2,
  kTetherBatteryTempHeatPlate1,
  kTetherBatteryTempHeatPlate2,
  kNumTetherBatteryTemps
} TetherBatteryTemp;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint8_t warning;
  uint8_t error;
  float lv_a;       // [V]
  float lv_b;       // [V]
  float v_lv_or;    // [V]
  float v_charger;  // [V]
  float i_hall;     // [A]
  float i_charger;  // [A]
  int16_t temps[kNumTetherBatteryTemps];
} TetherBatteryStatus;

typedef enum {
  kTetherCommsLinkPof       = 1 << 0,
  kTetherCommsLinkEop       = 1 << 1,
  kTetherCommsLinkWifi      = 1 << 2,
  kTetherCommsLinkLongRange = 1 << 3,
  kTetherCommsLinkJoystick  = 1 << 4,
} TetherCommsLink;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint8_t links_up;  // See TetherCommsLink enum.
  // Populated by CsA(digi) and CsB(microhard) [dBm].
  int16_t received_signal_strength;
} TetherCommsStatus;

typedef enum {
  kTetherMvlvTempEnclosureAir,
  kTetherMvlvTempFilterCap,
  kTetherMvlvTempHvResonantCap,
  kTetherMvlvTempIgbt,
  kTetherMvlvTempOutputSwitch,
  kTetherMvlvTempSyncRectMosfetSide,
  kTetherMvlvTempSyncRectMosfetTop,
  kTetherMvlvTempSyncRectPcb,
  kNumTetherMvlvTemps
} TetherMvlvTemp;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint8_t warning;
  uint8_t error;
  uint16_t status;
  float v_lv;      // [V]
  float v_lv_or;   // [V]
  float v_lv_pri;  // [V]
  float v_lv_sec;  // [V]
  float i_hall;    // [A]
  int16_t temps[kNumTetherMvlvTemps];
} TetherMvlvStatus;

typedef enum {
  kTetherNodeFlagSelfTestFailure = 1 << 0,
  kTetherNodeFlagPowerGood       = 1 << 1,
  kTetherNodeFlagNetworkAGood    = 1 << 2,
  kTetherNodeFlagNetworkBGood    = 1 << 3,
  kTetherNodeFlagAnyWarning      = 1 << 4,
  kTetherNodeFlagAnyError        = 1 << 5
} TetherNodeFlags;

typedef struct {
  int32_t no_update_count;  // Number of frame indices without update.
  uint8_t node;             // See AioNode enum.
  uint8_t flags;            // See TetherNodeFlags enum.
  uint8_t board_humidity;   // [percent, 0-100]
  int16_t board_temp;       // [C]
} TetherNodeStatus;

typedef struct {
  uint16_t frame_index;
  uint16_t received_frame_index;
  // Populated by CsA(digi) and CsB(microhard) [dBm].
  int16_t received_signal_strength;
  TetherControlCommand control_command;
  TetherControlTelemetry control_telemetry;
  TetherFlightComputer flight_computers[kNumFlightComputers];
  TetherGpsTime gps_time;
  TetherGpsStatus gps_statuses[kNumWingGpsReceivers];
  TetherCommsStatus comms_status;
  TetherMotorStatus motor_statuses[kNumMotors];
  TetherServoStatus servo_statuses[kNumServos];
  TetherReleaseStatus release_statuses[kNumLoadcellNodes];
  TetherBatteryStatus batt_a;  // TODO: Make array of batteries.
  TetherBatteryStatus batt_b;
  TetherMvlvStatus mvlv;
  TetherNodeStatus node_status;
} TetherDownMessage;

typedef enum {
  kTetherDownSourceForceSigned = -1,
  kTetherDownSourceCsA,
  kTetherDownSourceCsB,
  kTetherDownSourceCsGsA,
  kNumTetherDownSources
} TetherDownSource;

// TODO(b/70675299): This appears to require space for 1 bit more than
// go/makani-tether-message predicts.
typedef struct {
  uint8_t data[46];
} TetherDownPackedMessage;

typedef struct {
  uint16_t frame_index;
  int16_t received_signal_strength;  // Populated by CsA [dB].
  TetherJoystick joystick;
  TetherPlatform platform_a;
  TetherPlatform platform_b;
  TetherDrum drum_a;
  TetherDrum drum_b;
  TetherPlc plc;
  TetherGpsTime gps_time;
  TetherGpsStatus gps_status;
  TetherGroundStation ground_station;
  TetherGsGpsPosition gps_position;
  TetherGsGpsCompass gps_compass;
  TetherWind wind;
  TetherWeather weather;
  uint8_t rtcm[30];
} TetherUpMessage;

typedef enum {
  kTetherUpSourceForceSigned = -1,
  kTetherUpSourceCsA,
  kTetherUpSourceCsGsA,
  kTetherUpSourceCsGsB,
  kNumTetherUpSources
} TetherUpSource;

typedef struct {
  uint8_t data[98];
} TetherUpPackedMessage;

// Test infrastructure.

#define TEST_LIST_LENGTH 1024
#define TEST_NAME_LENGTH 64
#define TEST_INFO_LENGTH 64
#define TEST_CONDITION_LENGTH 128

// Message sent by host to execute a series of tests. The host should format
// the list of tests using strings "TestSuite.TestFunction", "TestSuite",
// or "TestFunction" to test a specific test suite or function. To run multiple
// tests, the list may contain a space or comma delimited list. Wildcards "*"
// and "?" may be used in the test suite or function name to match multiple
// tests as well.
typedef struct {
  AioNode node;                 // AIO node index.
  char list[TEST_LIST_LENGTH];  // List of tests to execute.
} TestExecuteMessage;

// Test result.
typedef struct {
  char suite[TEST_NAME_LENGTH];  // Suite name containing test function.
  char test[TEST_NAME_LENGTH];   // Test name (or function name).
  int32_t index;                 // Test number.
  int32_t failures;              // Number of failures experienced.
  int32_t runtime_usec;          // Elapsed test execution time.
} TestResult;

// Periodic status message. Sent between tests.
typedef struct {
  char node[TEST_NAME_LENGTH];  // Aio node name.
  BuildInfo build_info;   // Build info of target.
  uint8_t busy;           // Flag indicates more tests to execute.
  int32_t num_failures;   // Number of failures since TestExecuteMessage.
  TestResult result;      // Results of last test.
} TestStatusMessage;

// Sent upon test execution.
typedef struct {
  char suite[TEST_NAME_LENGTH];  // Suite name containing test function.
  char test[TEST_NAME_LENGTH];   // Test name (or function name).
  int32_t index;                 // Test number.
  int32_t timeout_usec;          // Test timeout parameter.
} TestStartMessage;

// Sent for each conditional macro failure.
typedef struct {
  int32_t index;  // Test number.
  char condition[TEST_CONDITION_LENGTH];  // Failing conditional expression.
  char file[TEST_INFO_LENGTH];  // Source file.
  char func[TEST_INFO_LENGTH];  // Function name.
  int32_t line;                 // Line number of failure.
  uint8_t value;                // Boolean value of evaluated condition.
} TestFailureMessage;

// Status message from dyno stand torque cell.
typedef struct {
  float torque;  // N-m.
  float angle;   // Radians.
  float omega;   // Radians per second.
} TorqueCellMessage;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_AVIONICS_MESSAGES_H_
