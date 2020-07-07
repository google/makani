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

#include "avionics/ground_power/q7/avionics_interface.h"

#include <malloc.h>
#include <mqueue.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/ground_power/q7/inverter_types.h"
#include "avionics/network/aio_node.h"

extern mqd_t g_inverter_command_queues[kNumInverters];

void EnqueueGroundPowerCommand() {
  GroundPowerCommandMessage command_message;

  if (CvtGetGroundPowerCommandMessage(kAioNodeOperator, &command_message, NULL,
                                      NULL)) {
    GroundPowerCommandQueueMessage message;
    message.message_type = kInverterMessageTypeCommand;
    message.message_union.command_message = command_message;

    // Place message in queue. If queue is full, return immediately.
    mq_send(g_inverter_command_queues[command_message.id],
            (const char *)&message, sizeof(GroundPowerCommandQueueMessage), 0);
  }
}

void EnqueueGroundPowerGetParam() {
  GroundPowerGetParamMessage get_param_message;
  if (CvtGetGroundPowerGetParamMessage(kAioNodeOperator, &get_param_message,
                                       NULL, NULL)) {
    GroundPowerCommandQueueMessage message;
    message.message_type = kInverterMessageTypeGetParam;
    message.message_union.get_param_message = get_param_message;

    // Place message in queue. If queue is full, return immediately.
    mq_send(g_inverter_command_queues[get_param_message.id],
            (const char *)&message, sizeof(GroundPowerCommandQueueMessage), 0);
  }
}

void EnqueueGroundPowerSetParam() {
  GroundPowerSetParamMessage set_param_message;
  if (CvtGetGroundPowerSetParamMessage(kAioNodeOperator, &set_param_message,
                                       NULL, NULL)) {
    GroundPowerCommandQueueMessage message;
    message.message_type = kInverterMessageTypeSetParam;
    message.message_union.set_param_message = set_param_message;

    // Place message in queue. If queue is full, return immediately.
    mq_send(g_inverter_command_queues[set_param_message.id],
            (const char *)&message, sizeof(GroundPowerCommandQueueMessage), 0);
  }
}

bool GetLoadbankSetLoad(int32_t *desired_net_load_kw) {
  LoadbankSetLoadMessage set_load_message;
  if (CvtGetLoadbankSetLoadMessage(kAioNodeOperator, &set_load_message,
                                   NULL, NULL)) {
    *desired_net_load_kw = set_load_message.desired_load_kw;
    return true;
  }
  return false;
}

bool GetLoadbankSetState(bool *loadbank_activated) {
  LoadbankSetStateMessage set_state_message;
  if (CvtGetLoadbankSetStateMessage(kAioNodeOperator, &set_state_message,
                                    NULL, NULL)) {
    *loadbank_activated = set_state_message.activate_loadbank;
    return true;
  }
  return false;
}
