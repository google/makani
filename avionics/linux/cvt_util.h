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

#ifndef AVIONICS_LINUX_CVT_UTIL_H_
#define AVIONICS_LINUX_CVT_UTIL_H_

#include <stdint.h>
#include <stdlib.h>  // For size_t.

#include "avionics/common/cvt.h"
#include "avionics/linux/aio.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"

// Unpack the next update to a specified CVT entry.
//
// If the entry is not already updated, receives incoming AIO messages for up to
// timeout_sec seconds looking for an update.  Other messages that arrive during
// this period are placed in the CVT.
//
// Args:
//   source        AioNode index into the CVT.
//   type          MessageType index into the CVT.
//   get_func      Function that unpacks a message of type T from the CVT.
//   dest          T* into which the message is unpacked.
//   timeout_sec   Number of seconds to wait for an updated message to arrive.
//                 If 0.0, the AIO port will still be checked once for new data.
//
// Returns:
//   True if an updated message was found/received before the timeout.
template <typename T>
bool CvtGetNextUpdate(AioNode source,
                      bool (*get_func)(AioNode, T *, uint16_t *, int64_t *),
                      double timeout_sec,
                      T *dest, uint16_t *sequence, int64_t *timestamp) {
  if (get_func(source, dest, sequence, timestamp)) return true;

  int64_t timeout_us = static_cast<int64_t>(timeout_sec * 1e6);
  int64_t end_ts = ClockGetUs() + timeout_us;
  for (int64_t cvt_timeout = timeout_us; cvt_timeout >= 0L;
       cvt_timeout = end_ts - ClockGetUs()) {
    if (AioRecvToCvt(cvt_timeout, NULL, NULL) != -1
        && get_func(source, dest, sequence, timestamp)) {
      return true;
    }
  }

  return false;
}

template <typename T>
bool CvtGetNextUpdate(AioNode source,
                      bool (*get_func)(AioNode, T *, uint16_t *, int64_t *),
                      double timeout_sec, T *dest) {
  return CvtGetNextUpdate(source, get_func, timeout_sec, dest,
                          nullptr, nullptr);
}


#endif  // AVIONICS_LINUX_CVT_UTIL_H_
