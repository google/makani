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

#include "control/avionics/avionics_sim.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_tether_message.h"
#include "avionics/common/tether_convert.h"
#include "avionics/linux/aio.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/message_type.h"
#include "system/labels.h"
#include "system/labels_util.h"

const MessageType *GetSubscribeTypesForSimulation(
    const HitlConfiguration *hitl_config, int32_t *num_subscribe_types) {
  static MessageType subscribe_types[kNumMessageTypes];
  memset(subscribe_types, 0, sizeof(subscribe_types));

  *num_subscribe_types = 0;
  subscribe_types[(*num_subscribe_types)++] = kMessageTypeControllerSync;
  subscribe_types[(*num_subscribe_types)++] = kMessageTypeFlightCommand;
  subscribe_types[(*num_subscribe_types)++] = kMessageTypeSimSensor;
  subscribe_types[(*num_subscribe_types)++] = kMessageTypeSimCommand;
  subscribe_types[(*num_subscribe_types)++] = kMessageTypeGroundEstimate;

  if (!hitl_config->use_software_joystick) {
    subscribe_types[(*num_subscribe_types)++] = kMessageTypeJoystickStatus;
  }

  return subscribe_types;
}

bool HitlConfigurationsEquivalent(const HitlConfiguration *a,
                                  const HitlConfiguration *b) {
  if (a->sim_level != b->sim_level ||
      a->use_software_joystick != b->use_software_joystick ||
      a->motor_level != b->motor_level) {
    return false;
  }

  for (int32_t i = 0; i < kNumServos; ++i) {
    if (a->servo_levels[i] != b->servo_levels[i]) return false;
  }

  return true;
}

void UpdateControllerCvtFromSimSensorMessage(
    const SimSensorMessage *sensor_message,
    const HitlConfiguration *hitl_config, uint16_t sequence,
    int64_t timestamp) {
  if (hitl_config->use_software_joystick &&
      sensor_message->control_input_messages_updated.joystick) {
    CvtPutJoystickStatusMessage(
        kAioNodeJoystickA, &sensor_message->control_input_messages.joystick,
        sequence, timestamp);
  }

  for (int32_t i = 0; i < kNumControllers; ++i) {
    if (sensor_message->control_input_messages_updated.controller_sync[i]) {
      ControllerLabel label = (ControllerLabel)i;
      CvtPutControllerSyncMessage(
          ControllerLabelToControllerAioNode(label),
          &sensor_message->control_input_messages.controller_sync[i], sequence,
          timestamp);
    }
  }

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    AioNode gps_node = WingGpsReceiverLabelToAioNode(i);
    GpsReceiverType gps_type = WingGpsReceiverLabelToGpsReceiverType(i);
    if (gps_type == kGpsReceiverTypeNovAtel) {
      if (sensor_message->control_input_messages_updated.wing_gps_novatel[i]) {
        CvtPutNovAtelSolutionMessage(
            gps_node,
            &sensor_message->control_input_messages.wing_gps_novatel[i],
            sequence, timestamp);
      }
    } else if (gps_type == kGpsReceiverTypeSeptentrio) {
      if (sensor_message->control_input_messages_updated
              .wing_gps_septentrio[i]) {
        CvtPutSeptentrioSolutionMessage(
            gps_node,
            &sensor_message->control_input_messages.wing_gps_septentrio[i],
            sequence, timestamp);
      }
    } else {
      assert(!(bool)"Invalid GPS type.");
    }
  }

  for (int32_t i = 0; i < kNumFlightComputers; ++i) {
    FlightComputerLabel label = (FlightComputerLabel)i;
    if (sensor_message->control_input_messages_updated.flight_comp_imus[i]) {
      CvtPutFlightComputerImuMessage(
          FlightComputerLabelToFlightComputerAioNode(label),
          &sensor_message->control_input_messages.flight_comp_imus[i], sequence,
          timestamp);
    }
    if (sensor_message->control_input_messages_updated.flight_comp_sensors[i]) {
      CvtPutFlightComputerSensorMessage(
          FlightComputerLabelToFlightComputerAioNode(label),
          &sensor_message->control_input_messages.flight_comp_sensors[i],
          sequence, timestamp);
    }
  }

  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    if (sensor_message->control_input_messages_updated.loadcell_messages[i]) {
      CvtPutLoadcellMessage(
          LoadcellNodeLabelToLoadcellNodeAioNode(i),
          &sensor_message->control_input_messages.loadcell_messages[i],
          sequence, timestamp);
    }
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    if (sensor_message->control_input_messages_updated.motor_statuses[i]) {
      CvtPutMotorStatusMessage(
          MotorLabelToMotorAioNode(i),
          &sensor_message->control_input_messages.motor_statuses[i], sequence,
          timestamp);
    }
  }

  for (int32_t i = 0; i < kNumServos; ++i) {
    if (sensor_message->control_input_messages_updated.servo_statuses[i]) {
      CvtPutServoStatusMessage(
          ServoLabelToServoAioNode(i),
          &sensor_message->control_input_messages.servo_statuses[i], sequence,
          timestamp);
    }
  }

  for (int32_t i = 0; i < kNumTetherUpSources; ++i) {
    if (sensor_message->control_input_messages_updated.tether_up_messages[i]) {
      TetherUpMessage in =
          sensor_message->control_input_messages.tether_up_messages[i];
      if (!hitl_config->use_software_joystick) {
        in.joystick.no_update_count = INT32_MAX;
      }

      CvtPutTetherUpMessage(TetherUpSourceToAioNode(i), &in, sequence,
                            timestamp);
    }
  }
}

void UpdateGroundEstimatorCvtFromSimSensorMessage(
    const SimSensorMessage *sensor_message, uint16_t sequence,
    int64_t timestamp) {
  if (sensor_message->ground_input_messages_updated.ground_comp_imus[0]) {
    CvtPutFlightComputerImuMessage(
        kAioNodeGpsBaseStation,
        &sensor_message->ground_input_messages.ground_comp_imus[0], sequence,
        timestamp);
  }

  if (sensor_message->ground_input_messages_updated.ground_comp_sensors[0]) {
    CvtPutFlightComputerSensorMessage(
        kAioNodeGpsBaseStation,
        &sensor_message->ground_input_messages.ground_comp_sensors[0], sequence,
        timestamp);
  }

  if (sensor_message->ground_input_messages_updated.ground_gps[0]) {
    CvtPutNovAtelSolutionMessage(
        kAioNodeGpsBaseStation,
        &sensor_message->ground_input_messages.ground_gps[0], sequence,
        timestamp);
  }

  if (sensor_message->ground_input_messages_updated.ground_compass[0]) {
    CvtPutNovAtelCompassMessage(
        kAioNodeGpsBaseStation,
        &sensor_message->ground_input_messages.ground_compass[0], sequence,
        timestamp);
  }
}
