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

#ifndef NAV_AIO_AIO_TO_INS_AIO_TO_INS_H_
#define NAV_AIO_AIO_TO_INS_AIO_TO_INS_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/aio_header.h"
#include "avionics/network/message_info.h"
#include "nav/aio/aio_to_ins/types.h"
#include "nav/ins/messages/messages.h"

#ifdef __cplusplus
extern "C" {
#endif

bool AioHeaderToInsMessageHeader(const AioSourceToInsLabelMap *map,
                                 int64_t timestamp, const AioHeader *in,
                                 InsMessageType type, InsMessageHeader *out);

bool AioMessageToInsMessages(const AioSourceToInsLabelMap *map,
                             int64_t timestamp, const AioHeader *header,
                             const AioMessageData *data,
                             NewInsMessageFunction func, void *arg);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // NAV_AIO_AIO_TO_INS_AIO_TO_INS_H_
