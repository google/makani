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

#include "nav/aio/aio_to_ins/flight_computer_sensor.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/aio_header.h"
#include "avionics/common/avionics_messages.h"
#include "common/macros.h"
#include "nav/aio/aio_to_ins/aio_to_ins.h"
#include "nav/aio/aio_to_ins/types.h"
#include "nav/ins/messages/message_types.h"
#include "nav/ins/messages/messages.h"

static void ImuConingScullingDataToInsInertialMessage(
    const ImuConingScullingData *in, InsInertialMessage *out) {
  out->dt = in->dt;
  out->phi[0] = in->phi[0];
  out->phi[1] = in->phi[1];
  out->phi[2] = in->phi[2];
  out->dvsf[0] = in->dvsf[0];
  out->dvsf[1] = in->dvsf[1];
  out->dvsf[2] = in->dvsf[2];
  out->alpha[0] = in->alpha[0];
  out->alpha[1] = in->alpha[1];
  out->alpha[2] = in->alpha[2];
  out->nu[0] = in->nu[0];
  out->nu[1] = in->nu[1];
  out->nu[2] = in->nu[2];
}

static bool ImuConingScullingDataToInsMessages(
    const AioSourceToInsLabelMap *map, int64_t timestamp,
    const AioHeader *header, int32_t num_datas,
    const ImuConingScullingData data[], NewInsMessageFunction func, void *arg) {
  // Indicate the input message incorporation status to the caller for
  // post-processing.
  bool used = false;
  // Insert oldest message first. InsInertial de-duplicates by seq_num
  // during the append operation.
  for (int32_t i = num_datas - 1; i >= 0; --i) {
    uint32_t delta_time = data[0].timestamp - data[i].timestamp;
    int64_t sample_time = timestamp - delta_time - data[i].latency;

    InsMessage m;
    if (AioHeaderToInsMessageHeader(map, sample_time, header,
                                    kInsMessageTypeInertial, &m.header)) {
      m.header.seq_num = (uint16_t)(header->sequence - i);
      ImuConingScullingDataToInsInertialMessage(data, &m.u.inertial);
      func(&m, arg);
      used = true;
    }
  }
  return used;
}

// Public functions.

bool FlightComputerSensorToInsMessages(const AioSourceToInsLabelMap *map,
                                       int64_t timestamp,
                                       const AioHeader *header,
                                       const FlightComputerSensorMessage *data,
                                       NewInsMessageFunction func, void *arg) {
  // Indicate the input message incorporation status to the caller for
  // post-processing.
  bool used = false;

  // Process inertial data.
  used |= ImuConingScullingDataToInsMessages(map, timestamp, header,
                                             ARRAYSIZE(data->cs), data->cs,
                                             func, arg);

  // TODO: Insert other data fields from FlightComputerSensorMessage.
  return used;
}
