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

#ifndef AVIONICS_FIRMWARE_DRIVERS_BCM53101_H_
#define AVIONICS_FIRMWARE_DRIVERS_BCM53101_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/network_config.h"
#include "avionics/common/network_diag_types.h"
#include "avionics/firmware/drivers/bcm.h"
#include "avionics/network/switch_types.h"

void Bcm53101Init(bool reset);
bool Bcm53101MirrorEnable(bool enable);
void Bcm53101Poll(const SwitchConfig *config);
bool Bcm53101Ready(void);
bool Bcm53101ReadMibCounter(PortStatistic statistic, int32_t port,
                            MibCounts counts[NUM_SWITCH_PORTS_BCM53101]);
bool Bcm53101ReadLinkStatus(uint32_t *link_status_bits,
                            uint32_t *down_ports_changed,
                            uint32_t *down_event_counter);
void Bcm53101ActivePortList(int32_t active_ports[], int32_t *number_of_ports);
void Bcm53101ReconfigureOptions(void);
const AddressRouteEntry *Bcm53101DumpRoutes(bool *finished);

#endif  // AVIONICS_FIRMWARE_DRIVERS_BCM53101_H_
