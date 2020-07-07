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

#ifndef AVIONICS_GROUND_POWER_Q7_GROUND_POWER_H_
#define AVIONICS_GROUND_POWER_Q7_GROUND_POWER_H_

#include <mqueue.h>
#include <stdbool.h>

#include "avionics/ground_power/q7/ground_power_types.h"

void GroundPowerInit(GroundPowerState *gp_state);
void GroundPowerStep(GroundPowerState *gp_state);
static void CreateMessageQueues(const char *queue_name,
                                struct mq_attr *queue_attr,
                                int32_t num_queues,
                                mqd_t message_queue_array[],
                                uint16_t flags);

#endif  // AVIONICS_GROUND_POWER_Q7_GROUND_POWER_H_
