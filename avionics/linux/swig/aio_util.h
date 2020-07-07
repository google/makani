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

// Header for functions to be wrapped into Python.

#ifndef AVIONICS_LINUX_SWIG_AIO_UTIL_H_
#define AVIONICS_LINUX_SWIG_AIO_UTIL_H_

#include <stddef.h>
#include <stdint.h>

#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the webmonitor AIO loop.
//
// Args:
//   node: Identity of the receiver.
//   port: Port number.
//   subscribe_types: Array of MessageTypes to be received.
//   num_subscribe_types: Number of subscribe_types.
int32_t InitAioLoop(AioNode node, uint16_t port,
                    const MessageType *subscribe_types,
                    int32_t num_subscribe_types);

// Tear down the webmonitor AIO loop.
void TearDownAioLoop(void);

// Repeatedly receive AIO messages and put them to CVT.
//
// Args:
//   timeout_us: Timeout (in us) when attempting to receive a message.
//   duration_us: Duration (in us) to capture messages.
//
// Returns:
//   The number of messages received.
size_t AioLoop(int64_t timeout_us, int64_t duration_us, bool use_sim_messages);

// Reset the AIO update information.
void ClearAioUpdates(void);

// Get AIO updates information.
//
// NOTE: Fixed width types like int32_t are not used, so that SWIG can use
// numpy.i to wrap the function. In addition, num_aio_nodes and
// num_message_types are really global constants, but we have to pass them
// in this format so that SWIG can recognize the pattern and wrap it properly.
//
// Args:
//   aio_updates: The output 2D array of size kNumAioNodes x kNumMessageTypes.
//       It will store the number of messages received since the last call to
//       ClearAioUpdates per AIO node and message type.
//   num_aio_nodes: The total number of AIO nodes. Must equal kNumAioNodes.
//   num_message_types: The total number of message types. Must equal
//       kNumMessageTypes.
//
// NOLINTNEXTLINE(runtime/int)
void GetAioUpdates(unsigned long long *aio_updates, int num_aio_nodes,
                   int num_message_types);

// Check whether we have reveived a particular message type.
//
// Args:
//   message_type: The message type to check.
//
// Returns:
//   True if the message has been updated.
bool IsMessageTypeUpdated(MessageType message_type);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_LINUX_SWIG_AIO_UTIL_H_
