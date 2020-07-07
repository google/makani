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

#include "avionics/firmware/network/net_aio.h"

#include <stdint.h>

#include "avionics/common/cvt.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"

// Public interface implementation.
void HandleAioFrame(AioNode source, MessageType type,
                    const void *buf, int32_t len,
                    uint16_t sequence, int64_t timestamp) {
  CvtPutPacked(source, type, NULL, buf, len, sequence, timestamp);
}
