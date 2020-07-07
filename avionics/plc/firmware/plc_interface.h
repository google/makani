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

#ifndef AVIONICS_PLC_FIRMWARE_PLC_INTERFACE_H_
#define AVIONICS_PLC_FIRMWARE_PLC_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/tether_message_types.h"

#define PLC_COMMS_VERSION 0
#define PLC_IP_ADDRESS    ((IpAddress) {192, 168, 1, 100})
#define UDP_PORT_PLC      2175

typedef struct {
  bool allow_arm_without_480v;
  ActuatorState detwist_state;
  uint16_t plc_command_sequence;
  EthernetAddress plc_mac;
  int64_t operator_cmd_timestamp;
  int64_t plc_status_timestamp;
  PlcStatusMessage plc_status;
  TetherDownMergeState tether_state;
  int64_t tether_state_timestamp;
  int64_t wakeup_plc_command;
} PlcInterfaceState;

void PlcInterfaceInit(int64_t reference_time);
void PlcPollInput(int64_t now);
bool HandlePlcMessage(uint16_t dest_port, int32_t length,
                      const uint8_t *data, int64_t now);

#endif  // AVIONICS_PLC_FIRMWARE_PLC_INTERFACE_H_
