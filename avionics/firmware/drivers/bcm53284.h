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

#ifndef AVIONICS_FIRMWARE_DRIVERS_BCM53284_H_
#define AVIONICS_FIRMWARE_DRIVERS_BCM53284_H_

#include <stdbool.h>

#include "avionics/common/network_diag_types.h"
#include "avionics/firmware/drivers/bcm.h"
#include "avionics/network/route_config.h"
#include "avionics/network/switch_types.h"

void Bcm53284Init(bool reset);
void Bcm53284Poll(const SwitchConfig *config);
bool Bcm53284Ready(void);
void Bcm53284MirrorEnable(bool enable);
bool Bcm53284ReadMibCounter(PortStatistic statistic, int32_t port,
                            MibCounts counts[NUM_SWITCH_PORTS_BCM53284]);
bool Bcm53284ReadLinkStatus(uint32_t *link_status_bits,
                            uint32_t *down_ports_changed,
                            uint32_t *down_port_events);
void Bcm53284ReconfigureOptions(void);
const AddressRouteEntry *Bcm53284DumpRoutes(bool *finished);

#endif  // AVIONICS_FIRMWARE_DRIVERS_BCM53284_H_
