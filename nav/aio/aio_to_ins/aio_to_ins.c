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

#include "nav/aio/aio_to_ins/aio_to_ins.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/aio_header.h"
#include "avionics/network/message_info.h"
#include "common/macros.h"
#include "nav/aio/aio_to_ins/flight_computer_sensor.h"

bool AioHeaderToInsMessageHeader(const AioSourceToInsLabelMap *map,
                                 int64_t timestamp, const AioHeader *in,
                                 InsMessageType type, InsMessageHeader *out) {
  assert(0 <= type && type < ARRAYSIZE(map->label));
  assert(0 <= in->source && in->source < ARRAYSIZE(map->label[type]));

  out->timestamp = timestamp;
  out->label = map->label[type][in->source];
  out->seq_num = in->sequence;
  out->type = type;

  return map->valid[type][in->source];
}

bool AioMessageToInsMessages(const AioSourceToInsLabelMap *map,
                             int64_t timestamp, const AioHeader *header,
                             const AioMessageData *data,
                             NewInsMessageFunction func, void *arg) {
  switch (header->type) {
    case kMessageTypeFlightComputerSensor:
      return FlightComputerSensorToInsMessages(map, timestamp, header,
                                               &data->flight_computer_sensor,
                                               func, arg);
    default:
      return false;
  }
}
