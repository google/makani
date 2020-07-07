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

// Avionics I/O.
//
// This is the communications layer used by nodes on the High Availability
// Multicast (HMC) network.
//
// HMC implements a publish-subscribe system using multicast. There is a 1:1
// mapping between message types and multicast groups. Senders publish messages
// to the corresponding multicast groups and HMC routes the messages to the
// appropriate destinations.

#if !defined(__linux__)
#error "This AIO implementation should only be used for Linux targets."
#endif

#ifndef AVIONICS_LINUX_AIO_H_
#define AVIONICS_LINUX_AIO_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/aio_header.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/linux/clock.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"

#ifdef __cplusplus
extern "C" {
#endif

// The maximum aio message length is computed from the Maximum Transmission
// Unit (MTU) for Ethernet v2 and the headers used for the IP and UDP protocols,
// and the maximum number of fragments we allow. The MTU = 1500, the IP header
// is 20 bytes, the UDP header is 8 bytes. That leaves 1472 bytes for the
// AIO message in each unfragmented frame.

#define AIO_MAX_FRAGMENTS 8
#define AIO_MAX_MESSAGE_LEN (1472 * AIO_MAX_FRAGMENTS - sizeof(AioHeader))

// Setup the AIO connection.  Returns -1 if an error occurred.
//
// node                 Identity of this connection.
// port                 Port number.
// subscribe_types      Array of MessageTypes to be received by this node.
// num_subscribe_types  Number of subscribe_types.
int32_t AioSetup(AioNode node, uint16_t port,
                 const MessageType *subscribe_types,
                 int32_t num_subscribe_types);

// Close the AIO connection.  Returns -1 if an error occurred.
int32_t AioClose(void);

// Receive the next message from an AIO connection and store it in
// the CVT.  If a timeout or an error occurred (e.g. an invalid
// message was received), returns -1. If a message is received and its type is
// subscribed, return 1; otherwise, return 0.
//
// timeout_us  Number of microseconds to wait before timing out.
// source      If not NULL, gets the source of the received subscribed message.
// type        If not NULL, gets the type of the received subscribed message.
int32_t AioRecvToCvt(int64_t timeout_us, AioNode *source, MessageType *type);

// Return the number of microseconds spent waiting either for new
// messages to arrive or for the appropriate time to run the user
// function.  This value is reset after each user function call.
int64_t AioGetIdleUsec(void);

// Sends an AIO message with an arbitrary AioHeader.  This is only for use by
// debugging tools!
// Returns -1 if an error occurred.
//
// type        Message type.
// buf         Pointer to message data.
// len         Length of message data.
// header      AIO header structure.
int32_t AioSendInternal(MessageType type, const void *buf, int32_t len,
                        const AioHeader *header);

// Sends an AIO message with a given type to all subscribed nodes.
// Returns -1 if an error occurred.
//
// type        Message type.
// buf         Pointer to message data.
// len         Length of message data.
int32_t AioSend(MessageType type, const void *buf, int32_t len);

// AioLoopStart is responsible for:
//
// - Setting up an AIO connection.
//
// - Periodically running a user-supplied function at a user-supplied period.
//
// - Receiving messages and storing them in the current value table (CVT).
//
// func        Main loop function. Should return as quickly as possible without
//                 blocking or busy-waiting.  Returns false if the loop should
//                 terminate.
// arg         Sole argument to 'func'.
// period_us   Period in microseconds between calls to 'func'.
int32_t AioLoopStart(AioNode node, uint16_t port,
                     const MessageType *subscribe_types,
                     int32_t num_subscribe_types,
                     bool (*func)(void *), void *arg, int64_t period_us);

// Get AIO statistics since last call.
void AioGetStats(AioStats *stats);

// Pack a structure into a buffer and send it over AIO.
//
// This is a macro to allow basic polymorphism on the signature of the
// packing functions and the type of the data to be packed (denoted by
// Type below).
//
// type           MessageType being sent.
// pack_func      Function of signature size_t(const Type*, size_t, uint8_t*)
//                    that packs the data.
// packed_length  Length of the structure after packing.
// unpacked_ptr   Pointer of a value of type Type to be transmitted.
#define AIO_SEND_PACKED(type, pack_func, packed_length, unpacked_ptr)   \
  do {                                                                  \
    uint8_t AIO_SEND_PACKED_buf[packed_length];                         \
    size_t AIO_SEND_PACKED_num_packed = pack_func(unpacked_ptr, 1U,     \
                                                  AIO_SEND_PACKED_buf); \
    assert(packed_length == AIO_SEND_PACKED_num_packed);                \
    (void)AIO_SEND_PACKED_num_packed;                                   \
    int32_t rc = AioSend(type, AIO_SEND_PACKED_buf, packed_length);     \
    assert(rc != -1);                                                   \
    (void)rc;                                                           \
  } while (0)

// Macro version of CvtGetNextUpdate (see avionics/linux/cvt_util.h).
//
// If the entry is not already updated, receives incoming AIO messages for up to
// timeout_sec seconds looking for an update.  Other messages that arrive during
// this period are placed in the CVT.
//
// source        AioNode index into the CVT.
// get_func      A CvtGet<...> function of signature:
//                     bool(AioNode, Type *, uint8_t *, int64_t *)
//                   that unpacks a message of type Type from the CVT.
// timeout_sec   [double] Number of seconds to wait for an updated message to
//               arrive.  If 0.0, the AIO port will still be checked once for
//               new data.
// unpacked_ptr  Pointer of type Type to unpack into.
// updated       Output pointer to a bool that indicates whether the CVT entry
//                   was updated.
//
// TODO: Move the visualizer and flight_gear_interface to
// C++ so this macro can be eliminated.
#define CVT_GET_NEXT_UPDATE(source, get_func, timeout_sec,      \
                            dest, updated)                      \
  do {                                                          \
    *updated = get_func(source, dest,  NULL, NULL);             \
    if (*updated) {                                             \
      break;                                                    \
    }                                                           \
                                                                \
    int64_t CVT_GET_timeout_us = (int64_t)(timeout_sec * 1e6);  \
    int64_t CVT_GET_end_ts = ClockGetUs() + CVT_GET_timeout_us; \
    for (; CVT_GET_timeout_us >= 0L;                            \
         CVT_GET_timeout_us = CVT_GET_end_ts - ClockGetUs()) {  \
      if (AioRecvToCvt(CVT_GET_timeout_us, NULL, NULL) != -1    \
          && get_func(source, dest, NULL, NULL)) {              \
        *updated = true;                                        \
        break;                                                  \
      }                                                         \
    }                                                           \
  } while (0)

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_LINUX_AIO_H_
