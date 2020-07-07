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

#ifndef AVIONICS_CS_FIRMWARE_TETHER_COMMS_H_
#define AVIONICS_CS_FIRMWARE_TETHER_COMMS_H_

#include <stdint.h>

#include "avionics/firmware/monitors/cs_types.h"
#include "avionics/firmware/network/net_mon_types.h"
#include "avionics/network/aio_node.h"

void TetherCommsInit(uint16_t network_id);
void TetherCommsPoll(AioNode me, const CsMonitorData *cs_mon,
                     const CoreSwitchStats *switch_stats,
                     int16_t microhard_rssi);

#endif  // AVIONICS_CS_FIRMWARE_TETHER_COMMS_H_
