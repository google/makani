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

#include "avionics/common/tether_convert.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/gill_types.h"
#include "avionics/common/novatel_types.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/motor/firmware/flags.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_stats.h"
#include "common/c_math/util.h"
#include "common/macros.h"

#define GPS_TIME_MAX_LATENCY_US (1 * 1000 * 1000)
#define GPS_TIME_MAX_NO_UPDATE_COUNT (60 * TETHER_UP_FREQUENCY_HZ)

static void IncrementNoUpdateCount(bool updated, int32_t *no_update_count) {
  if (updated) {
    *no_update_count = 0;
  } else if (*no_update_count < INT32_MAX) {
    ++(*no_update_count);
  }
}

static void SetFlag(uint8_t mask, bool set, uint8_t *flags) {
  if (set) {
    *flags |= mask;
  } else {
    *flags &= (uint8_t)~mask;
  }
}

static uint16_t AioSequenceToTetherSequence(uint16_t sequence) {
  return sequence % TETHER_SEQUENCE_ROLLOVER;
}

static uint8_t FloatToUint8(float in) {
  return (uint8_t)roundf(Saturatef(in, 0.0f, (float)UINT8_MAX));
}

static int16_t FloatToInt16(float in) {
  return (int16_t)roundf(Saturatef(in, (float)INT16_MIN, (float)INT16_MAX));
}

static TetherGpsSolutionStatus NovAtelSolutionTypeToTetherGpsSolutionStatus(
    NovAtelSolutionType novatel) {
  switch (novatel) {
    case kNovAtelSolutionTypeNone:
      return kTetherGpsSolutionStatusNone;
    case kNovAtelSolutionTypeFixedPos:
      return kTetherGpsSolutionStatusFixedPos;
    case kNovAtelSolutionTypeFixedHeight:
    case kNovAtelSolutionTypeSingle:
      return kTetherGpsSolutionStatusSingle;
    case kNovAtelSolutionTypePsrdiff:
      return kTetherGpsSolutionStatusDifferential;
    case kNovAtelSolutionTypeWaas:
      return kTetherGpsSolutionStatusSbasAided;
    case kNovAtelSolutionTypeL1Float:
    case kNovAtelSolutionTypeIonofreeFloat:
    case kNovAtelSolutionTypeNarrowFloat:
      return kTetherGpsSolutionStatusRtkFloat;
    case kNovAtelSolutionTypeL1Int:
    case kNovAtelSolutionTypeWideInt:
    case kNovAtelSolutionTypeNarrowInt:
      return kTetherGpsSolutionStatusRtkFixed;
    case kNovAtelSolutionTypeDopplerVelocity:
    case kNovAtelSolutionTypePropagated:
    case kNovAtelSolutionTypeOmnistar:
    case kNovAtelSolutionTypeRtkDirectIns:
    case kNovAtelSolutionTypeOmnistarHp:
    case kNovAtelSolutionTypeOmnistarXp:
    case kNovAtelSolutionTypeCdgps:
    default:
      return kTetherGpsSolutionStatusSingle;  // Not supported.
  }
}

static TetherGpsSolutionStatus SeptentrioPvtModeToTetherGpsSolutionStatus(
    SeptentrioPvtMode mode) {
  switch (mode) {
    case kSeptentrioPvtModeNoSolution:
      return kTetherGpsSolutionStatusNone;
    case kSeptentrioPvtModeStandAlone:
      return kTetherGpsSolutionStatusSingle;
    case kSeptentrioPvtModeDifferential:
      return kTetherGpsSolutionStatusDifferential;
    case kSeptentrioPvtModeFixedLocation:
      return kTetherGpsSolutionStatusFixedPos;
    case kSeptentrioPvtModeRtkFixed:
      return kTetherGpsSolutionStatusRtkFixed;
    case kSeptentrioPvtModeRtkFloat:
      return kTetherGpsSolutionStatusRtkFloat;
    case kSeptentrioPvtModeSbasAided:
      return kTetherGpsSolutionStatusSbasAided;
    case kSeptentrioPvtModeMovingBaseRtkFixed:
    case kSeptentrioPvtModeMovingBaseRtkFloat:
    case kSeptentrioPvtModePrecisePointPositioning:
    default:
      return kTetherGpsSolutionStatusSingle;  // Not supported.
  }
}

static void UpdateNodeStatusBoardTempHumidity(float temp, float humidity,
                                              TetherNodeStatus *node_status) {
  node_status->board_temp = FloatToInt16(temp);
  node_status->board_humidity = FloatToUint8(humidity);
}

static void UpdateNodeStatusWarningFlag(bool has_warning,
                                        TetherNodeStatus *node_status) {
  // Cleared on transmission. See TetherNodeStatusToOutputTetherNodeStatus.
  if (has_warning) {
    node_status->flags |= kTetherNodeFlagAnyWarning;
  }
}

static void UpdateNodeStatusErrorFlag(bool has_error,
                                      TetherNodeStatus *node_status) {
  // Cleared on transmission. See TetherNodeStatusToOutputTetherNodeStatus.
  if (has_error) {
    node_status->flags |= kTetherNodeFlagAnyError;
  }
}

static void UpdateNodeStatusFromAioModuleMonitorData(
    const AioModuleMonitorData *aio_mon, TetherNodeStatus *node_status) {
  if (CheckWarning(&aio_mon->flags, kAioMonitorWarning12v)
      || CheckWarning(&aio_mon->flags, kAioMonitorWarning1v2)
      || CheckWarning(&aio_mon->flags, kAioMonitorWarning2v5)
      || CheckWarning(&aio_mon->flags, kAioMonitorWarning3v3)) {
    node_status->flags &= (uint8_t)~kTetherNodeFlagPowerGood;
  }

  UpdateNodeStatusWarningFlag(HasWarning(&aio_mon->flags), node_status);
  UpdateNodeStatusErrorFlag(HasError(&aio_mon->flags), node_status);
  if (aio_mon->si7021_populated & (1 << kAioSi7021MonitorBoard)) {
    const Si7021OutputData *in = &aio_mon->si7021_data[kAioSi7021MonitorBoard];
    UpdateNodeStatusBoardTempHumidity(in->temperature, in->rel_humidity,
                                      node_status);
  }
}

static bool IsAccessSwitchLinkUp(const AccessSwitchStats *as, int32_t port) {
  if (as != NULL) {
    assert(0 <= port && port < ARRAYSIZE(as->stats));
    return (as->link_status_bits & (1U << port))
        && as->stats[port].rx_multicast_packet_rate > 0;
  }
  return false;
}

static bool IsCoreSwitchLinkUp(const CoreSwitchStats *cs, int32_t port) {
  if (cs != NULL) {
    assert(0 <= port && port < ARRAYSIZE(cs->stats));
    return (cs->link_status_bits & (1U << port))
        && cs->stats[port].rx_multicast_packet_rate > 0;
  }
  return false;
}

// TETHER_UP_PERIOD_US and TETHER_DOWN_PERIOD_US represent the Ethernet
// transmit period as defined by network.yaml.
COMPILE_ASSERT(TETHER_UP_PERIOD_US == TETHER_DOWN_PERIOD_US,
               TetherUp_and_TetherDown_must_operate_at_the_same_frequency);
COMPILE_ASSERT(TETHER_UP_PERIOD_US % 1000 == 0,
               TetherUp_period_must_be_a_multiple_of_1_millisecond);

void BatteryStatusMessageToTetherBatteryStatus(
    const BatteryStatusMessage *in, TetherNodeStatus *node_status,
    TetherBatteryStatus *out) {
  assert(node_status != NULL);
  assert(out != NULL);
  if (in != NULL) {
    out->warning = (HasWarning(&in->aio_mon.flags)
                    || HasWarning(&in->batt_mon.flags));
    out->error = HasError(&in->aio_mon.flags) || HasError(&in->batt_mon.flags);
    out->lv_a = in->batt_mon.analog_data[kBattAnalogVoltageLvA];
    out->lv_b = in->batt_mon.analog_data[kBattAnalogVoltageLvB];
    out->v_lv_or = in->batt_mon.analog_data[kBattAnalogVoltageVLvOr];
    out->v_charger =
        in->batt_mon.ltc4151_data[kBattLtc4151MonitorChargerOutput].voltage;
    out->i_hall = in->batt_mon.analog_data[kBattAnalogVoltageIHall];
    out->i_charger =
        in->batt_mon.ltc4151_data[kBattLtc4151MonitorChargerOutput].current;
    out->temps[kTetherBatteryTempBattery1] = FloatToInt16(
        in->batt_mon.mcp342x_data[kBattMcp342xMonitorBatteries1]);
    out->temps[kTetherBatteryTempBattery2] = FloatToInt16(
        in->batt_mon.mcp342x_data[kBattMcp342xMonitorBatteries2]);
    out->temps[kTetherBatteryTempHeatPlate1] = FloatToInt16(
        in->batt_mon.mcp342x_data[kBattMcp342xMonitorHeatPlate1]);
    out->temps[kTetherBatteryTempHeatPlate2] = FloatToInt16(
        in->batt_mon.mcp342x_data[kBattMcp342xMonitorHeatPlate2]);

    UpdateNodeStatusWarningFlag(HasWarning(&in->batt_mon.flags), node_status);
    UpdateNodeStatusErrorFlag(HasError(&in->batt_mon.flags), node_status);
    UpdateNodeStatusFromAioModuleMonitorData(&in->aio_mon, node_status);
  }
  IncrementNoUpdateCount(in != NULL, &node_status->no_update_count);
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void ControllerCommandMessageToTetherControlCommand(
    ControllerLabel lead_controller, const ControllerCommandMessage *in,
    uint16_t sequence, TetherControlCommand *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->controller_label = (uint8_t)lead_controller;
    out->detwist_angle = (float)in->detwist_position;
    out->winch_velocity = (float)in->winch_velocity;
    out->gs_mode_request = (uint8_t)in->gs_mode_request;
    out->gs_unpause_transform = (uint8_t)in->gs_unpause_transform;
    out->gs_azi_target = in->gs_azi_target;
    out->gs_azi_dead_zone = in->gs_azi_dead_zone;
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void CoreSwitchStatsToTetherCommsStatus(
    AioNode source, const CoreSwitchStats *in, int16_t long_range_rssi,
    TetherCommsStatus *out) {
  SetFlag(kTetherCommsLinkPof, IsCoreSwitchLinkUp(in, 20), &out->links_up);
  SetFlag(kTetherCommsLinkEop, false, &out->links_up);
  SetFlag(kTetherCommsLinkJoystick, IsCoreSwitchLinkUp(in, 23), &out->links_up);

  if (source == kAioNodeCsA) {
    SetFlag(kTetherCommsLinkWifi, IsCoreSwitchLinkUp(in, 18), &out->links_up);
  } else if (source == kAioNodeCsB) {
    SetFlag(kTetherCommsLinkWifi, IsCoreSwitchLinkUp(in, 22), &out->links_up);
  } else {
    assert(false);
  }

  // RSSI should be negative dBm.
  SetFlag(kTetherCommsLinkLongRange, long_range_rssi < 0, &out->links_up);
  out->received_signal_strength = long_range_rssi;  // [dBm]
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void CsMonitorDataToTetherNodeStatus(
    const CsMonitorData *in, TetherNodeStatus *node_status) {
  assert(node_status != NULL);
  if (in != NULL) {
    UpdateNodeStatusWarningFlag(HasWarning(&in->flags), node_status);
    UpdateNodeStatusErrorFlag(HasError(&in->flags), node_status);

    if (in->si7021_populated & (1 << kCsSi7021MonitorBoard)) {
      const Si7021OutputData *data = &in->si7021_data[kCsSi7021MonitorBoard];
      UpdateNodeStatusBoardTempHumidity(data->temperature, data->rel_humidity,
                                        node_status);
    }
  }
  IncrementNoUpdateCount(in != NULL, &node_status->no_update_count);
}

void DrumSensorsMessageToTetherDrum(
    const DrumSensorsMessage *in, uint16_t sequence, TetherDrum *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->gsg_axis1 = WrapAngle(in->encoders.gsg_azi);
    out->gsg_axis2 = WrapAngle(in->encoders.gsg_ele);
    SetFlag(
        kTetherDrumFlagGsgAxis1Fault,
        CheckWarning(&in->encoders.status, kGsDrumEncodersWarningGsgAzimuth)
        || CheckError(&in->encoders.status, kGsDrumEncodersErrorGsgAzimuth),
        &out->flags);
    SetFlag(
        kTetherDrumFlagGsgAxis2Fault,
        CheckWarning(&in->encoders.status, kGsDrumEncodersWarningGsgElevation)
        || CheckError(&in->encoders.status, kGsDrumEncodersErrorGsgElevation),
        &out->flags);
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void FlightComputerSensorMessageToTetherNodeStatus(
    const FlightComputerSensorMessage *in, TetherNodeStatus *node_status) {
  assert(node_status != NULL);
  if (in != NULL) {
    UpdateNodeStatusWarningFlag(
        HasWarning(&in->flags) || HasWarning(&in->fc_mon.flags), node_status);
    UpdateNodeStatusErrorFlag(
        HasError(&in->flags) || HasError(&in->fc_mon.flags), node_status);
    UpdateNodeStatusFromAioModuleMonitorData(&in->aio_mon, node_status);
  }
  IncrementNoUpdateCount(in != NULL, &node_status->no_update_count);
}

void FlightComputerSensorMessageToTetherFlightComputer(
    const FlightComputerSensorMessage *in, TetherFlightComputer *out) {
  assert(out != NULL);
  if (in != NULL) {
    SetFlag(kTetherFlightComputerFlagImuGood,
            !CheckWarning(&in->flags, kFlightComputerWarningImu)
            && !CheckWarning(&in->flags, kFlightComputerWarningImuData),
            &out->flags);
    SetFlag(kTetherFlightComputerFlagGpsGood,
            !CheckWarning(&in->flags, kFlightComputerWarningGps), &out->flags);
    SetFlag(kTetherFlightComputerFlagPitotGood,
            !CheckWarning(&in->flags, kFlightComputerWarningPitotAltitude)
            && !CheckWarning(&in->flags, kFlightComputerWarningPitotPitch)
            && !CheckWarning(&in->flags, kFlightComputerWarningPitotSpeed)
            && !CheckWarning(&in->flags, kFlightComputerWarningPitotYaw),
            &out->flags);
    SetFlag(kTetherFlightComputerFlagFpvEnabled,
            CheckWarning(&in->flags, kFlightComputerWarningFpvEnabled),
            &out->flags);
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void GpsTimeMessageToTetherGpsTime(
    const GpsTimeMessage *in, int64_t latency_usec, TetherGpsTime *out) {
  assert(out != NULL);

  // Only reset no_update_count for valid GpsTimeMessages.
  bool updated = false;

  // On initialization, latency_usec could be huge.
  if (0 <= latency_usec && latency_usec < GPS_TIME_MAX_LATENCY_US) {
    // Time to add to the reported time_of_week.
    int32_t adjust_usec = (int32_t)latency_usec;

    // Reset TetherGpsTime when the GpsTimeMessage update is valid.
    if (in != NULL && 0 <= in->latency && in->latency < GPS_TIME_MAX_LATENCY_US
        && in->time_of_week < TETHER_GPS_TIME_OF_WEEK_ROLLOVER) {
      adjust_usec += in->latency;
      out->time_of_week = in->time_of_week;
      updated = true;  // To reset no_update_count.
    }

    // Propagate TetherGpsTime between GpsTimeMessage updates.
    if (out->time_of_week < TETHER_GPS_TIME_OF_WEEK_INVALID
        && out->no_update_count < GPS_TIME_MAX_NO_UPDATE_COUNT) {
      out->time_of_week += adjust_usec / 1000;  // [ms]
      out->time_of_week %= TETHER_GPS_TIME_OF_WEEK_ROLLOVER;
    } else {
      out->time_of_week = TETHER_GPS_TIME_OF_WEEK_INVALID;
    }
  } else {
    out->time_of_week = TETHER_GPS_TIME_OF_WEEK_INVALID;
  }
  IncrementNoUpdateCount(updated, &out->no_update_count);
}

void GroundStationPlcStatusMessageToTetherPlc(
    const GroundStationPlcStatusMessage *in, uint16_t sequence,
    TetherPlc *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->detwist_angle = (float)in->plc.detwist_position;
    out->drum_angle = 0.0f;  // Not supported by top hat configuration.
    out->proximity = 0x0;    // Not supported by top hat configuration.
    SetFlag(kTetherPlcFlagPlcWarning, in->plc.warning_flags != 0x0,
            &out->flags);
    SetFlag(kTetherPlcFlagPlcError, in->plc.error_flags != 0x0, &out->flags);
    SetFlag(kTetherPlcFlagDetwistFault, false, &out->flags);
    SetFlag(kTetherPlcFlagDrumFault, false, &out->flags);
    SetFlag(kTetherPlcFlagProximityFault, false, &out->flags);
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void GroundStationWeatherMessageToTetherWeather(
    const GroundStationWeatherMessage *in, uint16_t sequence,
    TetherWeather *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->pressure_pa = in->weather.pressure * 100.0f;  // Convert hPa to Pa.
    out->humidity = in->weather.humidity;
    out->temperature = in->weather.temperature;
    SetFlag(kTetherWeatherFlagFault,
            in->weather.status != kGillMetPakStatusOk
            && in->weather.status != kGillMetPakStatusAcceptableData,
            &out->flags);
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void GroundStationWeatherMessageToTetherWind(
    const GroundStationWeatherMessage *in, uint16_t sequence,
    TetherWind *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->velocity[0] = in->wind.wind_velocity[0];
    out->velocity[1] = in->wind.wind_velocity[1];
    out->velocity[2] = in->wind.wind_velocity[2];
    if (in->wind.status == kGillWindmasterStatusOk) {
      out->status = kTetherWindStatusGood;
    } else if (in->wind.status == kGillWindmasterStatusAtMaxGain
               || in->wind.status == kGillWindmasterStatusRetriesUsed) {
      out->status = kTetherWindStatusWarning;
    } else {
      out->status = kTetherWindStatusFault;
    }
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void JoystickStatusMessageToTetherJoystick(
    const JoystickStatusMessage *in, uint16_t sequence, TetherJoystick *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->roll = in->roll;
    out->pitch = in->pitch;
    out->yaw = in->yaw;
    out->throttle = in->throttle;
    out->tri_switch = in->tri_switch;
    out->momentary_switch = in->momentary_switch;
    out->tether_release_interlock_code = in->tether_release_interlock_code;
    out->scuttle_code = in->scuttle_code;
    SetFlag(kTetherJoystickFlagFault,
            CheckWarning(&in->status, kJoystickWarningNotPresent),
            &out->flags);
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void LoadcellMessageToTetherReleaseStatus(
    const LoadcellMessage *in, TetherNodeStatus *node_status,
    TetherReleaseStatus *out) {
  assert(node_status != NULL);
  assert(out != NULL);
  if (in != NULL) {
    out->state = (uint8_t)in->tether_release_state;
    out->interlock_switched = in->tether_release_fully_armed;
    out->released = in->tether_released;

    UpdateNodeStatusWarningFlag(HasWarning(&in->loadcell_mon.flags)
                                | HasWarning(&in->status), node_status);
    UpdateNodeStatusErrorFlag(HasError(&in->loadcell_mon.flags)
                              | HasError(&in->status), node_status);
    UpdateNodeStatusFromAioModuleMonitorData(&in->aio_mon, node_status);
  }
  IncrementNoUpdateCount(in != NULL, &node_status->no_update_count);
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void MotorStatusMessageUpdateTetherMotorStatus(
    const MotorStatusMessage *in, TetherNodeStatus *node_status,
    TetherMotorStatus *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->warning = (in->motor_warning != 0x0);
    out->error = (in->motor_error != 0x0);
    out->state = (uint8_t)in->state;
    out->speed = FloatToInt16(in->omega);
    out->iq = FloatToInt16(in->iq);
    out->id = FloatToInt16(in->id);
    out->bus_voltage = FloatToInt16(in->bus_voltage);
    out->bus_current = FloatToInt16(in->bus_current);

    out->motor_temps[kTetherMotorTempNacelleAir] =
        FloatToInt16(in->temps[kMotorThermalChannelNacelleAir]);
    out->motor_temps[kTetherMotorTempRotor] =
        FloatToInt16(in->temps[kMotorThermalChannelRotor]);
    out->motor_temps[kTetherMotorTempStatorCoil] =
        FloatToInt16(in->temps[kMotorThermalChannelStatorCoil]);
    out->motor_temps[kTetherMotorTempStatorCore] =
        FloatToInt16(in->temps[kMotorThermalChannelStatorCore]);

    out->controller_temps[kTetherMotorControllerTempAir] =
        FloatToInt16(in->temps[kMotorThermalChannelControllerAir]);
    out->controller_temps[kTetherMotorControllerTempBoard] =
        FloatToInt16(in->temps[kMotorThermalChannelBoard]);
    out->controller_temps[kTetherMotorControllerTempCapacitor] =
        FloatToInt16(in->temps[kMotorThermalChannelCapacitor]);
    out->controller_temps[kTetherMotorControllerTempHeatPlate] =
        FloatToInt16(in->temps[kMotorThermalChannelHeatPlate1]);

    UpdateNodeStatusWarningFlag(in->motor_warning != 0x0, node_status);
    UpdateNodeStatusErrorFlag(in->motor_error != 0x0, node_status);
    if (in->motor_mon.si7021_populated & (1 << kMotorSi7021MonitorBoard)) {
      const Si7021OutputData *data =
          &in->motor_mon.si7021_data[kMotorSi7021MonitorBoard];
      UpdateNodeStatusBoardTempHumidity(data->temperature, data->rel_humidity,
                                        node_status);
    } else {
      UpdateNodeStatusBoardTempHumidity(in->temps[kMotorThermalChannelBoard],
                                        0.0f, node_status);
    }
  }
  IncrementNoUpdateCount(in != NULL, &node_status->no_update_count);
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void GroundStationStatusMessageToTetherGroundStation(
    const GroundStationStatusMessage *in, uint16_t sequence,
    TetherGroundStation *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->mode = in->status.mode;
    out->transform_stage = in->status.transform_stage;
    out->drum_angle = (float)in->status.winch.position;
    out->detwist_angle = (float)in->status.detwist.position;
    out->proximity = (uint8_t)in->status.bridle_proximity.proximity;
    out->tether_engaged = in->status.tether_engagement.engaged;
    SetFlag(kTetherGroundStationFlagError, HasError(&in->status.flags),
            &out->flags);
    SetFlag(kTetherGroundStationFlagDetwistError,
            HasError(&in->status.detwist.flags), &out->flags);
    SetFlag(kTetherGroundStationFlagDrumError,
            HasError(&in->status.winch.flags), &out->flags);
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void MvlvStatusMessageUpdateTetherMvlvStatus(
    const MvlvStatusMessage *in, TetherNodeStatus *node_status,
    TetherMvlvStatus *out) {
  assert(node_status != NULL);
  if (in != NULL) {
    out->warning = (HasWarning(&in->aio_mon.flags)
                    || HasWarning(&in->mvlv_mon.flags));
    out->error = HasError(&in->aio_mon.flags) || HasError(&in->mvlv_mon.flags);
    out->v_lv = in->mvlv_mon.analog_data[kMvlvAnalogVoltageVLv];
    out->v_lv_or = in->mvlv_mon.analog_data[kMvlvAnalogVoltageVLvOr];
    out->v_lv_pri = in->mvlv_mon.analog_data[kMvlvAnalogVoltageVLvPri];
    out->v_lv_sec = in->mvlv_mon.analog_data[kMvlvAnalogVoltageVLvSec];
    out->i_hall = in->mvlv_mon.analog_data[kMvlvAnalogVoltageIHall];
    out->status = in->mvlv_mon.flags.status;
    out->temps[kTetherMvlvTempEnclosureAir] = FloatToInt16(
        in->mvlv_mon.mcp342x_data[kMvlvMcp342xMonitorEnclosureAir]);
    out->temps[kTetherMvlvTempFilterCap] = FloatToInt16(
        in->mvlv_mon.mcp342x_data[kMvlvMcp342xMonitorFilterCap]);
    out->temps[kTetherMvlvTempHvResonantCap] = FloatToInt16(
        in->mvlv_mon.mcp342x_data[kMvlvMcp342xMonitorHvResonantCap]);
    out->temps[kTetherMvlvTempIgbt] = FloatToInt16(
        in->mvlv_mon.mcp342x_data[kMvlvMcp342xMonitorIgbt]);
    out->temps[kTetherMvlvTempOutputSwitch] = FloatToInt16(
        in->mvlv_mon.mcp342x_data[kMvlvMcp342xMonitorOutputSwitch]);
    out->temps[kTetherMvlvTempSyncRectMosfetSide] = FloatToInt16(
        in->mvlv_mon.mcp342x_data[kMvlvMcp342xMonitorSyncRectMosfetSide]);
    out->temps[kTetherMvlvTempSyncRectMosfetTop] = FloatToInt16(
        in->mvlv_mon.mcp342x_data[kMvlvMcp342xMonitorSyncRectMosfetTop]);
    out->temps[kTetherMvlvTempSyncRectPcb] = FloatToInt16(
        in->mvlv_mon.mcp342x_data[kMvlvMcp342xMonitorSyncRectPcb]);
    UpdateNodeStatusWarningFlag(HasWarning(&in->mvlv_mon.flags), node_status);
    UpdateNodeStatusErrorFlag(HasError(&in->mvlv_mon.flags), node_status);
    UpdateNodeStatusFromAioModuleMonitorData(&in->aio_mon, node_status);
  }
  IncrementNoUpdateCount(in != NULL, &node_status->no_update_count);
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void NovAtelCompassMessageToTetherGsGpsCompass(
    const NovAtelCompassMessage *in, uint16_t sequence,
    TetherGsGpsCompass *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->heading = WrapAngle(in->heading.heading);
    out->heading_sigma = in->heading.heading_sigma;
    out->heading_rate = in->heading_rate.heading_rate;
    SetFlag(kTetherGsGpsCompassFlagFault,
            in->heading.pos_sol_status != kNovAtelSolutionStatusSolComputed
            || in->heading.pos_type == kNovAtelSolutionTypeNone,
            &out->flags);
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void NovAtelSolutionMessageToTetherGpsPosition(
    const NovAtelSolutionMessage *in, uint16_t sequence,
    TetherGsGpsPosition *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->ecef[0] = in->best_xyz.pos_x;
    out->ecef[1] = in->best_xyz.pos_y;
    out->ecef[2] = in->best_xyz.pos_z;
    SetFlag(kTetherGsGpsPositionFlagFault,
            in->best_xyz.pos_sol_status != kNovAtelSolutionStatusSolComputed
            || in->best_xyz.pos_type == kNovAtelSolutionTypeNone,
            &out->flags);
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void NovAtelSolutionMessageToTetherGpsStatus(
    const NovAtelSolutionMessage *in, uint16_t sequence,
    TetherGpsStatus *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->satellites = in->best_xyz.num_sol;
    out->status = NovAtelSolutionTypeToTetherGpsSolutionStatus(
        (NovAtelSolutionType)in->best_xyz.pos_type);
    out->pos_sigma =
        sqrtf(in->best_xyz.pos_x_sigma * in->best_xyz.pos_x_sigma
              + in->best_xyz.pos_y_sigma * in->best_xyz.pos_y_sigma
              + in->best_xyz.pos_z_sigma * in->best_xyz.pos_z_sigma);
    if (0.0f <= in->avg_cn0 && in->avg_cn0 <= 127.0f) {
      out->avg_cn0 = (int8_t)in->avg_cn0;
    } else {
      out->avg_cn0 = 0;  // Invalid.
    }
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void PlatformSensorsMessageToTetherPlatform(
    const PlatformSensorsMessage *in, uint16_t sequence, TetherPlatform *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->perch_azi = WrapAngle(in->encoders.perch_azi);
    SetFlag(
        kTetherPlatformFlagPerchAzimuthFault,
        CheckWarning(&in->encoders.status, kGsPerchEncodersWarningPerchAzimuth)
        || CheckError(&in->encoders.status, kGsPerchEncodersErrorPerchAzimuth),
        &out->flags);

    out->levelwind_ele = WrapAngle(in->encoders.levelwind_ele);
    SetFlag(
        kTetherPlatformFlagLevelwindElevationFault,
        CheckWarning(&in->encoders.status,
                     kGsPerchEncodersWarningLevelwindElevation)
        || CheckError(&in->encoders.status,
                      kGsPerchEncodersErrorLevelwindElevation),
        &out->flags);
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void RecorderStatusMessageUpdateTetherRecorderStatus(
    const RecorderStatusMessage *in, TetherNodeStatus *node_status) {
  assert(node_status != NULL);
  if (in != NULL) {
    UpdateNodeStatusWarningFlag(HasWarning(&in->recorder_mon.flags),
                                node_status);
    UpdateNodeStatusErrorFlag(HasError(&in->recorder_mon.flags),
                              node_status);
    UpdateNodeStatusFromAioModuleMonitorData(&in->aio_mon, node_status);
  }
  IncrementNoUpdateCount(in != NULL, &node_status->no_update_count);
}

void SelfTestMessageUpdateTetherNodeStatus(
    const SelfTestMessage *in, TetherNodeStatus *node_status) {
  assert(node_status != NULL);

  if (in != NULL) {
    node_status->flags |= kTetherNodeFlagSelfTestFailure;
    node_status->no_update_count = 0;
  }
}

void SeptentrioSolutionMessageToTetherGpsStatus(
    const SeptentrioSolutionMessage *in, uint16_t sequence,
    TetherGpsStatus *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->sequence = AioSequenceToTetherSequence(sequence);
    out->satellites = in->pvt_cartesian.nr_sv;
    out->status = SeptentrioPvtModeToTetherGpsSolutionStatus(
        (SeptentrioPvtMode)(in->pvt_cartesian.mode
                            & kSeptentrioPvtModeBitSolutionMask));
    out->pos_sigma = sqrtf(in->pos_cov_cartesian.cov_xx
                           + in->pos_cov_cartesian.cov_yy
                           + in->pos_cov_cartesian.cov_zz);
    if (0.0f <= in->avg_cn0 && in->avg_cn0 <= 127.0f) {
      out->avg_cn0 = (int8_t)in->avg_cn0;
    } else {
      out->avg_cn0 = 0;  // Invalid.
    }
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void ServoStatusMessageUpdateTetherServoStatus(
    const ServoStatusMessage *in, TetherNodeStatus *node_status,
    TetherServoStatus *out) {
  assert(out != NULL);
  if (in != NULL) {
    out->state = (uint8_t)in->state;
    out->angle = in->angle_estimate;
    out->r22_temp = in->r22.temperature;

    UpdateNodeStatusWarningFlag(HasWarning(&in->servo_mon.flags), node_status);
    UpdateNodeStatusErrorFlag(HasError(&in->servo_mon.flags), node_status);
    UpdateNodeStatusFromAioModuleMonitorData(&in->aio_mon, node_status);
  }
  IncrementNoUpdateCount(in != NULL, &node_status->no_update_count);
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void ShortStackStatusMessageUpdateTetherShortStackStatus(
    const ShortStackStatusMessage *in, TetherNodeStatus *node_status) {
  assert(node_status != NULL);
  if (in != NULL) {
    UpdateNodeStatusWarningFlag(HasWarning(&in->short_stack_mon.flags),
                                node_status);
    UpdateNodeStatusErrorFlag(HasError(&in->short_stack_mon.flags),
                              node_status);
    UpdateNodeStatusFromAioModuleMonitorData(&in->aio_mon, node_status);
  }
  IncrementNoUpdateCount(in != NULL, &node_status->no_update_count);
}

void SlowStatusMessageUpdateTetherNodeStatus(
    const SlowStatusMessage *in, TetherNodeStatus *node_status) {
  assert(node_status != NULL);

  if (in != NULL) {
    SetFlag(kTetherNodeFlagNetworkAGood,
            IsAccessSwitchLinkUp(&in->switch_stats, 0), &node_status->flags);
    SetFlag(kTetherNodeFlagNetworkBGood,
            IsAccessSwitchLinkUp(&in->switch_stats, 1), &node_status->flags);
  }
}

void SmallControlTelemetryMessageToTetherControlTelemetry(
    const SmallControlTelemetryMessage *in, uint16_t sequence,
    TetherControlTelemetry *out) {
  assert(out != NULL);
  if (in != NULL) {
    *out = *in;
    out->sequence = AioSequenceToTetherSequence(sequence);
  }
  IncrementNoUpdateCount(in != NULL, &out->no_update_count);
}

void TetherNodeStatusToOutputTetherNodeStatus(
    AioNode source, TetherNodeStatus *node_status, TetherNodeStatus *out) {
  // Output.
  node_status->node = (uint8_t)source;
  *out = *node_status;

  // Reset latching flags for next update.
  node_status->flags &= (uint8_t)~(kTetherNodeFlagSelfTestFailure
                                   | kTetherNodeFlagAnyWarning
                                   | kTetherNodeFlagAnyError);
}

// Helper functions.

int32_t TetherNoUpdateCountToMilliseconds(int32_t no_update_count) {
  return no_update_count * (TETHER_UP_PERIOD_US / 1000);
}

AioNode TetherDownSourceToAioNode(TetherDownSource source) {
  switch (source) {
    case kTetherDownSourceCsA:
      return kAioNodeCsA;
    case kTetherDownSourceCsB:
      return kAioNodeCsB;
    case kTetherDownSourceCsGsA:
      return kAioNodeCsGsA;
    case kTetherDownSourceForceSigned:
    case kNumTetherDownSources:
    default:
      assert(false);
      return kAioNodeUnknown;
  }
}

AioNode TetherUpSourceToAioNode(TetherUpSource source) {
  switch (source) {
    case kTetherUpSourceCsA:
      return kAioNodeCsA;
    case kTetherUpSourceCsGsA:
      return kAioNodeCsGsA;
    case kTetherUpSourceCsGsB:
      return kAioNodeCsGsB;
    case kTetherUpSourceForceSigned:
    case kNumTetherUpSources:
    default:
      assert(false);
      return kAioNodeUnknown;
  }
}
