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

#include "avionics/network/aio_latency.h"

#include <stdint.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/firmware/network/net_send.h"

void AioLatencyPoll(void) {
  LatencyResponseMessage resp_msg;
  LatencyProbeMessage probe_msg;
  if (CvtGetLatencyProbeMessage(kAioNodeOperator, &probe_msg, NULL, NULL)) {
    resp_msg.timestamp = probe_msg.timestamp;
    NetSendAioLatencyResponseMessage(&resp_msg);
  }
}
