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

#ifndef SYSTEM_LABELS_UTIL_H_
#define SYSTEM_LABELS_UTIL_H_

#include <stdbool.h>

#include "avionics/common/gps_receiver.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

// Converts a servo label to the corresponding flap label.  Note that
// multiple servos can attach to the same flap on the M600.
FlapLabel ServoToFlap(ServoLabel servo_label);

const char *LoadcellSensorLabelToString(LoadcellSensorLabel loadcell_label);

AioNode WingGpsReceiverLabelToAioNode(WingGpsReceiverLabel wing_gps_receiver);
GpsReceiverType WingGpsReceiverLabelToGpsReceiverType(
    WingGpsReceiverLabel wing_gps_receiver);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SYSTEM_LABELS_UTIL_H_
