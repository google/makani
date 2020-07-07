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

#include "avionics/firmware/network/switch_config.h"

#include <assert.h>
#include <string.h>

#include "avionics/firmware/identity/identity.h"
#include "avionics/network/route_config.h"
#include "avionics/network/switch_config.h"
#include "avionics/network/switch_info.h"
#include "avionics/network/switch_types.h"

static SwitchConfig g_config;

static MulticastTable g_mcast_table;
static SwitchOptions g_switch_options;

void SwitchConfigInit(void) {
  g_config = (SwitchConfig){GetSwitchInfo(AppConfigGetAioNode()),
                            &g_switch_options, &g_mcast_table};
  GetDefaultSwitchOptions(g_config.info, &g_switch_options);
  GenerateMulticastRoutingTable(AppConfigGetAioNode(), &g_mcast_table);
}

SwitchOptions *SwitchConfigGetSwitchOptions(void) {
  assert(g_config.info != NULL);
  return &g_switch_options;
}

const SwitchConfig *GetSwitchConfig(void) {
  assert(g_config.info != NULL);
  return &g_config;
}
