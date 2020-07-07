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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/network_config.h"
#include "avionics/ground_power/q7/ground_power.h"
#include "avionics/linux/aio.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"

GroundPowerState g_gp_state;

static const MessageType *GetSubscribeTypes(int32_t *num_subscribe_types) {
  static const MessageType kGroundPowerSubscribeTypes[] = {
    kMessageTypeGroundPowerCommand, kMessageTypeGroundPowerSetParam,
    kMessageTypeGroundPowerGetParam, kMessageTypeLoadbankSetLoad,
    kMessageTypeLoadbankSetState,
  };

  *num_subscribe_types = ARRAYSIZE(kGroundPowerSubscribeTypes);
  return kGroundPowerSubscribeTypes;
}

// Function called periodically by AIO.
static bool AioCallback(void *arg) {
  GroundPowerStep(&g_gp_state);
  return true;
}

// Entry point for the ground_power_monitor which runs on the q7.
int main(int argc, char **argv) {
  UNUSED(argc);
  UNUSED(argv);

  int32_t num_subscribe_types;
  const MessageType *subscribe_types = GetSubscribeTypes(&num_subscribe_types);

  GroundPowerInit(&g_gp_state);

  // TODO: Consider running the AioLoop more often but send the
  // ground power and loadbank messages only a fraction of the time.
  AioLoopStart(kAioNodeGroundPowerQ7A, UDP_PORT_AIO, subscribe_types,
               num_subscribe_types, AioCallback, NULL, 25000);
  AioClose();
  exit(EXIT_SUCCESS);
}
