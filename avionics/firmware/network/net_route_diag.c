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

#include "avionics/firmware/network/net_route_diag.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/bcm53284.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net_send.h"

static bool g_dump_routes = false;
static int16_t g_dump_routes_index = 0;

void NetRouteDiagInit(void) {
  g_dump_routes = false;
  g_dump_routes_index = 0;
}

void NetRouteDiagPoll(void) {
  DumpRoutesRequestMessage dump_routes_request;
  if (CvtGetDumpRoutesRequestMessage(kAioNodeOperator, &dump_routes_request,
                                     NULL, NULL)) {
    if (dump_routes_request.target == AppConfigGetAioNode() && !g_dump_routes) {
      g_dump_routes = true;
      g_dump_routes_index = 0;
    }
  }
  if (g_dump_routes) {
    DumpRoutesResponseMessage dump_routes_response;
    bool dump_routes_finished = false;
    const AddressRouteEntry *entry;
    if (BootConfigGetHardwareType() == kHardwareTypeCs) {
      entry = Bcm53284DumpRoutes(&dump_routes_finished);
    } else {
      entry = Bcm53101DumpRoutes(&dump_routes_finished);
    }
    if (entry != NULL) {
      dump_routes_response.index = g_dump_routes_index;
      dump_routes_response.entry = *entry;
      ++g_dump_routes_index;
      NetSendAioDumpRoutesResponseMessage(&dump_routes_response);
    }
    if (dump_routes_finished) {
      g_dump_routes = false;
    }
  }
}
