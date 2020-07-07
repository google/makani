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

#include "avionics/ground_power/q7/ground_power.h"

#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/ground_power/q7/avionics_interface.h"
#include "avionics/ground_power/q7/inverter.h"
#include "avionics/ground_power/q7/loadbank.h"
#include "avionics/ground_power/q7/loadbank_types.h"
#include "avionics/linux/aio.h"
#include "avionics/network/message_type.h"

InverterBank g_inverter_bank;
LoadbankSet g_loadbank_set;
SharkMeters g_shark_meters;
static pthread_t g_inverter_query_threads[kNumInverters];
static pthread_t g_loadbank_command_threads[kNumLoadbanks];
static pthread_t g_loadbank_relay_calc_thread;
static pthread_t g_loadbank_data_thread;

GroundPowerStatusMessage g_inverter_message[kNumInverters];
mqd_t g_inverter_data_queues[kNumInverters];
mqd_t g_inverter_command_queues[kNumInverters];
mqd_t g_inverter_ack_queues[kNumInverters];
mqd_t g_loadbank_command_queues[kNumLoadbanks];

void GroundPowerInit(GroundPowerState *gp_state) {
  memset(gp_state, 0, sizeof(*gp_state));
  InverterBankInit(&g_inverter_bank);
  LoadbankSetInit(&g_loadbank_set);
  SharkInit(&g_shark_meters);
  gp_state = kInitializationStateModbusTcpConnected;

  // Message queues for GroundPowerStatusMessages
  struct mq_attr inverter_data_queue_attr;
  inverter_data_queue_attr.mq_flags = 0;
  inverter_data_queue_attr.mq_maxmsg = 2;
  inverter_data_queue_attr.mq_msgsize = sizeof(GroundPowerStatusMessage);
  inverter_data_queue_attr.mq_curmsgs = 0;

  // Message queues for GroundPowerCommandMessages, GroundPowerSetParamMessages,
  // GroundPowerGetParamMessages
  struct mq_attr inverter_command_queue_attr;
  inverter_command_queue_attr.mq_flags = 0;
  inverter_command_queue_attr.mq_maxmsg = 2;
  inverter_command_queue_attr.mq_msgsize =
      sizeof(GroundPowerCommandQueueMessage);
  inverter_command_queue_attr.mq_curmsgs = 0;

  // Message queues for GroundPowerAckParamMessages.
  struct mq_attr inverter_ack_queue_attr;
  inverter_ack_queue_attr.mq_flags = 0;
  inverter_ack_queue_attr.mq_maxmsg = 2;
  inverter_ack_queue_attr.mq_msgsize =
      sizeof(GroundPowerAckParamMessage);
  inverter_ack_queue_attr.mq_curmsgs = 0;

  // Message queues for sending relay setting to load banks.
  struct mq_attr loadbank_command_queue_attr;
  loadbank_command_queue_attr.mq_flags = 0;
  loadbank_command_queue_attr.mq_maxmsg = 2;  // TODO: decrease 1?
  loadbank_command_queue_attr.mq_msgsize = (sizeof(uint8_t) *
                                            kLoadbankParamsRelaysPerBank);
  loadbank_command_queue_attr.mq_curmsgs = 0;

  CreateMessageQueues("/inverter", &inverter_data_queue_attr,
                      kNumInverters, g_inverter_data_queues,
                      O_CREAT | O_RDWR | O_NONBLOCK);

  CreateMessageQueues("/invrcmdq", &inverter_command_queue_attr,
                      kNumInverters, g_inverter_command_queues,
                      O_CREAT | O_RDWR | O_NONBLOCK);

  CreateMessageQueues("/invrackq", &inverter_ack_queue_attr,
                      kNumInverters, g_inverter_ack_queues,
                      O_CREAT | O_RDWR | O_NONBLOCK);

  CreateMessageQueues("/loadbankcmd", &loadbank_command_queue_attr,
                      kNumLoadbanks, g_loadbank_command_queues,
                      O_CREAT | O_RDWR | O_NONBLOCK);

  // Initialize g_inverter_message values.
  for (int32_t k = 0; k < kNumInverters; ++k) {
    GroundPowerStatusMessage init_status_message;
    init_status_message.id = (uint8_t)k;
    g_inverter_message[k] = init_status_message;
  }
}

static void CreateMessageQueues(const char *queue_name,
                                struct mq_attr *queue_attr,
                                int32_t num_queues, mqd_t message_queue_array[],
                                uint16_t flags) {
  mqd_t message_queue;

  for (int32_t j = 0; j < num_queues; ++j) {
    // Create inverter message queues.
    char message_queue_name[80];

    snprintf(message_queue_name, strlen(queue_name) + 4, "%s_%d", queue_name,
             j);
    message_queue = mq_open(message_queue_name, flags,
                            0644, queue_attr);

    if (message_queue == (mqd_t)-1) {
      perror("mq_open():");
      assert(!(bool)"GroundPowerInit(): Unable to open message queue.");
      exit(EXIT_FAILURE);
    }

    message_queue_array[j] = message_queue;
  }
}

static void StartInverterCommThreads(InverterBank *inverters) {
  Inverter *inverter;
  for (int32_t i = 0; i < kNumInverters; ++i) {
    inverter = &(inverters->inverter_array[i]);
    if (pthread_create(&g_inverter_query_threads[i], NULL,
                       InverterCommunication, inverter)) {
      assert(!(bool)"ground_power_monitor: Unable to create inverter threads.");
      exit(EXIT_FAILURE);
    }
  }
}

static void StartLoadbankThreads(LoadbankSet *loadbanks) {
  Loadbank *loadbank;
  for (int32_t i = 0; i < kNumLoadbanks; ++i) {
    loadbank = &(loadbanks->loadbank_array[i]);
    if (pthread_create(&g_loadbank_command_threads[i], NULL,
                       LoadbankCommandThread, loadbank)) {
      assert(
          !(bool)"ground_power_monitor: Unable to create loadbank cmd thread.");
      exit(EXIT_FAILURE);
    }
  }

  if (pthread_create(&g_loadbank_relay_calc_thread, NULL,
                     LoadbankRelayCalcThread, &g_shark_meters)) {
    assert(
        !(bool)"ground_power_monitor: Unable to create loadbank relay thread.");
    exit(EXIT_FAILURE);
  }

  if (pthread_create(&g_loadbank_data_thread, NULL,
                     LoadbankDataThread, &g_shark_meters)) {
    assert(
        !(bool)"ground_power_monitor: Unable to create loadbank data thread.");
    exit(EXIT_FAILURE);
  }
}

static void SendGroundPowerStatusMessage(void) {
  // Receive inverter message from queue.
  static int32_t inverter_index = 0;
  GroundPowerStatusMessage status_message;

  for (int32_t k = 0; k < kNumInverters; ++k) {
    ssize_t retval = mq_receive(g_inverter_data_queues[k],
                                (char *)&status_message,
                                sizeof(GroundPowerStatusMessage), 0);
    if (retval > 0) {
      g_inverter_message[k] = status_message;
    }
  }

  // Send message for one inverter.
  AIO_SEND_PACKED(kMessageTypeGroundPowerStatus, PackGroundPowerStatusMessage,
                  PACK_GROUNDPOWERSTATUSMESSAGE_SIZE,
                  &g_inverter_message[inverter_index]);

  // Cycle through inverters sequentially.
  inverter_index = (uint8_t)((inverter_index + 1) % kNumInverters);
}

static void SendGroundPowerAckParamMessage(void) {
  // Receive inverter ack param message from queue.
  GroundPowerAckParamMessage ack_message;

  for (int32_t k = 0; k < kNumInverters; ++k) {
    ssize_t retval = mq_receive(g_inverter_ack_queues[k],
                                (char *)&ack_message,
                                sizeof(GroundPowerAckParamMessage), 0);
    if (retval > 0) {
      // Send ack param over AIO network.
      AIO_SEND_PACKED(kMessageTypeGroundPowerAckParam,
                      PackGroundPowerAckParamMessage,
                      PACK_GROUNDPOWERACKPARAMMESSAGE_SIZE, &ack_message);
    }
  }
}

static void HandleLoadbankSetLoadMessage(void) {
  LoadbankAckParamMessage ack_message;
  if (GetLoadbankSetLoad(&g_shark_meters.status_message.desired_net_load_kw)) {
    ack_message.value = g_shark_meters.status_message.desired_net_load_kw;
    // Send ack param over AIO network.
    AIO_SEND_PACKED(kMessageTypeLoadbankAckParam, PackLoadbankAckParamMessage,
                    PACK_LOADBANKACKPARAMMESSAGE_SIZE, &ack_message);
  }
}

static void HandleLoadbankSetStateMessage(void) {
  LoadbankStateAckParamMessage ack_message;
  if (GetLoadbankSetState(&g_shark_meters.status_message.loadbank_activated)) {
    ack_message.value = g_shark_meters.status_message.loadbank_activated;
    // Send ack param over AIO network.
    AIO_SEND_PACKED(kMessageTypeLoadbankStateAckParam,
                    PackLoadbankStateAckParamMessage,
                    PACK_LOADBANKSTATEACKPARAMMESSAGE_SIZE, &ack_message);
  }
}

// Send info about loadbanks to AIO network.
static void SendLoadbankStatusMessage(void) {
  // Populate status message using atomic writes from thread-updated struct.
  // TODO: Add mutex to prevent loadbank data from updating while we
  // are copying to the LoadbankStatusMessage.
  LoadbankStatusMessage message;
  message.loadbank_activated = g_shark_meters.status_message.loadbank_activated;
  message.loadbank_power_modbus_status = g_shark_meters.loadbank_mb_status;
  message.kite_power_modbus_status = g_shark_meters.kite_mb_status;
  message.loadbank_cmd_modbus_status =
      ((uint32_t)(g_loadbank_set.loadbank_array[0].mb_status) << 0)
      | ((uint32_t)(g_loadbank_set.loadbank_array[1].mb_status) << 1);
  message.loadbank_power_kw = g_shark_meters.status_message.loadbank_power_kw;
  message.loadbank_kvar = g_shark_meters.status_message.loadbank_kvar;
  message.loadbank_kva = g_shark_meters.status_message.loadbank_kva;
  message.kite_power_kw = g_shark_meters.status_message.kite_power_kw;
  message.kite_kvar = g_shark_meters.status_message.kite_kvar;
  message.kite_kva = g_shark_meters.status_message.kite_kva;
  message.desired_net_load_kw =
      g_shark_meters.status_message.desired_net_load_kw;
  message.n_requested_relays = g_shark_meters.status_message.n_requested_relays;
  message.relay_mask = g_shark_meters.status_message.relay_mask;
  AIO_SEND_PACKED(kMessageTypeLoadbankStatus, PackLoadbankStatusMessage,
                  PACK_LOADBANKSTATUSMESSAGE_SIZE, &message);
}

void GroundPowerStep(GroundPowerState *gp_state) {
  switch (*gp_state) {
    case kInitializationStateModbusTcpConnected:
      StartInverterCommThreads(&g_inverter_bank);
      StartLoadbankThreads(&g_loadbank_set);
      *gp_state = kInitializationStateQueryThreadsRunning;
      return;

    case kInitializationStateQueryThreadsRunning: {
      // Enqueue GroundPowerCommandMessage if present.
      EnqueueGroundPowerCommand();

      // Enqueue GroundPowerGetParamMessage if present.
      EnqueueGroundPowerGetParam();

      // Enqueue GroundPowerSetParamMessage if present.
      EnqueueGroundPowerSetParam();

      // Store LoadbankSetLoadMessage data for thread if present.
      HandleLoadbankSetLoadMessage();

      // Store LoadbankSetStateMessage data for thread if present.
      HandleLoadbankSetStateMessage();

      // Send GroundPowerStatusMessage.
      SendGroundPowerStatusMessage();

      // Send GroundPowerAckParamMessage.
      SendGroundPowerAckParamMessage();

      // Send LoadbankStatusMessage.
      SendLoadbankStatusMessage();

      return;
    }
    default:
      assert(
          !(bool)"ground_power_monitor: unknown state in GroundPowerStep().");
      exit(EXIT_FAILURE);
  }
}
