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

#include "avionics/linux/swig/aio_util.h"

#include <assert.h>
#include <limits.h>
#include <memory.h>
#include <stdint.h>
#include <time.h>

#include "avionics/common/cvt.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/tether_convert.h"
#include "avionics/linux/aio.h"
#include "avionics/linux/clock.h"
#include "avionics/network/message_type.h"
#include "control/avionics/avionics_sim.h"
#include "control/system_params.h"
#include "gs/monitor2/high_frequency_filters/common.h"
#include "sim/pack_sim_messages.h"

int32_t InitAioLoop(AioNode node, uint16_t port,
                    const MessageType* subscribe_types,
                    int32_t num_subscribe_types) {
  ClearAioUpdates();
  InitFilters();
  return AioSetup(node, port, subscribe_types, num_subscribe_types);
}

void TearDownAioLoop(void) { TearDownFilters(); }

// Dictionary to store the accumulated number of messages received per
// AIO node and message type.
//
// TODO: This consumes more memory than necessary. Also, consider
// having a class to better abstract the interface.
//
// NOTE: Fixed width types like uint64_t are not used, so that SWIG can use
// numpy.i to wrap GetAioUpdates without jeopardizing memcpy.
//
// NOLINTNEXTLINE(runtime/int)
static unsigned long long g_aio_updates[kNumAioNodes][kNumMessageTypes];
static bool g_message_updates[kNumMessageTypes];

size_t AioLoop(int64_t timeout_us, int64_t duration_us, bool use_sim_messages) {
  // Compute initial wakeup time.
  int64_t startup = ClockGetUs();
  size_t message_count = 0L;

  // Begin main loop.
  // TODO: Revisit clock overflow.
  while (ClockGetUs() - startup < duration_us) {
    MessageType message_type;
    AioNode source;
    if (AioRecvToCvt(timeout_us, &source, &message_type) > 0) {
      if (g_aio_updates[source][message_type] < ULLONG_MAX) {
        g_aio_updates[source][message_type]++;
      }
      g_message_updates[message_type] = true;
      if (message_count < ULLONG_MAX) {
        message_count++;
      }

      // If using simulated messages, modify the CVT and update metadata
      // accordingly.
      if (use_sim_messages) {
        if (message_type == kMessageTypeSimSensor) {
          uint16_t sequence;
          int64_t timestamp;

          const uint8_t* buf =
              CvtPeek(source, message_type, &sequence, &timestamp);

          // TODO: Add fine-grain control over sim-populated messages,
          // for the case where there are both mocked and real messages.
          SimSensorMessage sensor_message;
          UnpackSimSensorMessage(buf, 1, &sensor_message);
          UpdateControllerCvtFromSimSensorMessage(&sensor_message,
                                                  &sensor_message.hitl_config,
                                                  sequence, timestamp);
        } else if (message_type == kMessageTypeSimTetherDown) {
          // TODO: Use high-frequency filters to merge
          // TetherDownMessages from different sources, and annotate when a
          // sub-message goes stale.
          uint16_t sequence;
          int64_t timestamp;

          const uint8_t* buf =
              CvtPeek(source, message_type, &sequence, &timestamp);

          SimTetherDownMessage sim_tether_down;
          UnpackSimTetherDownMessage(buf, 1, &sim_tether_down);
          for (int32_t i = 0; i < kNumTetherDownSources; ++i) {
            if (sim_tether_down.updated[i]) {
              CvtPutTetherDownMessage(TetherDownSourceToAioNode(i),
                                      &sim_tether_down.messages[i], sequence,
                                      timestamp);
            }
          }
        }
      }
    }
  }

  RunAllFilters();

  return message_count;
}

void ClearAioUpdates(void) {
  for (int32_t source = 0; source < kNumAioNodes; ++source) {
    for (int32_t type = 0; type < kNumMessageTypes; ++type) {
      g_aio_updates[source][type] = 0L;
    }
  }
  for (int32_t type = 0; type < kNumMessageTypes; ++type) {
    g_message_updates[type] = false;
  }
}

// NOLINTNEXTLINE(runtime/int)
void GetAioUpdates(unsigned long long* aio_updates, int num_aio_nodes,
                   int num_message_types) {
  assert(num_aio_nodes == kNumAioNodes);
  assert(num_message_types == kNumMessageTypes);
  memcpy(aio_updates, &g_aio_updates[0][0], sizeof(g_aio_updates));
}

bool IsMessageTypeUpdated(MessageType message_type) {
  return g_message_updates[message_type];
}
