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

#include "avionics/firmware/drivers/bcm_unified.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/bcm53284.h"
#include "avionics/network/switch_types.h"

#define RESET_CYCLES CLOCK32_MSEC_TO_CYCLES(100)

static enum {
  kStateReset,
  kStateResetWait,
  kStateDetect,
} g_state;

static SwitchType g_switch_type;

static void UnknownInit(bool reset) {
  (void)reset;
  assert(false);
}

static void UnknownPoll(const SwitchConfig *config);

static bool UnknownReady(void) {
  return false;
}

static const AddressRouteEntry *UnknownDumpRoutes(bool *finished) {
  *finished = true;
  return NULL;
}

static const struct {
  void (*const init)(bool);
  void (*const poll)(const SwitchConfig *);
  bool (*const ready)(void);
  const AddressRouteEntry * (*const dump_routes)(bool *);
} g_drivers[kNumSwitchTypes] = {
  [kSwitchTypeUnknown] = {
    UnknownInit,
    UnknownPoll,
    UnknownReady,
    UnknownDumpRoutes,
  },
  [kSwitchTypeBcm53101] = {
    Bcm53101Init,
    Bcm53101Poll,
    Bcm53101Ready,
    Bcm53101DumpRoutes,
  },
  [kSwitchTypeBcm53284] = {
    Bcm53284Init,
    Bcm53284Poll,
    Bcm53284Ready,
    Bcm53284DumpRoutes,
  }
};

static void UnknownPoll(const SwitchConfig *config) {
  static uint32_t reset_timeout;
  switch (g_state) {
    case kStateReset:
      if (config == NULL || config->info == NULL
          || config->info->type == kSwitchTypeUnknown) {
        BcmInit();
        BcmReset(true);
        g_state = kStateResetWait;
        reset_timeout = Clock32GetCycles() + RESET_CYCLES;
      } else {
        g_switch_type = config->info->type;
        g_drivers[g_switch_type].init(true);
      }
      break;
    case kStateResetWait:
      if (CLOCK32_GE(Clock32GetCycles(), reset_timeout)) {
        BcmReset(false);
        g_state = kStateDetect;
      }
      break;
    case kStateDetect:
      if (BcmDetectSwitchType(&g_switch_type)) {
        assert(g_switch_type == kSwitchTypeBcm53101 ||
               g_switch_type == kSwitchTypeBcm53284);
        g_drivers[g_switch_type].init(false);
      }
      break;
    default:
      assert(false);
  }
  BcmPoll();
}

void BcmUnifiedInit(bool reset) {
  if (reset) {
    g_state = kStateReset;
  } else {
    g_state = kStateDetect;
  }
  g_switch_type = kSwitchTypeUnknown;
}

void BcmUnifiedPoll(const SwitchConfig *config) {
  assert(g_switch_type == kSwitchTypeBcm53101 ||
         g_switch_type == kSwitchTypeBcm53284 ||
         g_switch_type == kSwitchTypeUnknown);
  g_drivers[g_switch_type].poll(config);
}

bool BcmUnifiedReady(void) {
  assert(g_switch_type == kSwitchTypeBcm53101 ||
         g_switch_type == kSwitchTypeBcm53284 ||
         g_switch_type == kSwitchTypeUnknown);
  return g_drivers[g_switch_type].ready();
}

const AddressRouteEntry *BcmUnifiedDumpRoutes(bool *finished) {
  assert(g_switch_type == kSwitchTypeBcm53101 ||
         g_switch_type == kSwitchTypeBcm53284 ||
         g_switch_type == kSwitchTypeUnknown);
  return g_drivers[g_switch_type].dump_routes(finished);
}
