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

#ifndef AVIONICS_SERVO_FIRMWARE_INPUT_CVT_H_
#define AVIONICS_SERVO_FIRMWARE_INPUT_CVT_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"

bool CvtGetClearErrorLogMessage(AioNode src, ServoClearErrorLogMessage *msg,
                                uint16_t *sequence, int64_t *timestamp);
bool CvtGetGetParamMessage(AioNode src, ServoGetParamMessage *msg,
                           uint16_t *sequence, int64_t *timestamp);
bool CvtGetSetParamMessage(AioNode src, ServoSetParamMessage *msg,
                           uint16_t *sequence, int64_t *timestamp);
bool CvtGetSetStateMessage(AioNode src, ServoSetStateMessage *msg,
                           uint16_t *sequence, int64_t *timestamp);

#endif  // AVIONICS_SERVO_FIRMWARE_INPUT_CVT_H_
