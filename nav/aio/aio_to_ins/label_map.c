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

#include "nav/aio/aio_to_ins/label_map.h"

#include <assert.h>
#include <stdbool.h>

#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "nav/ins/messages/labels.h"
#include "nav/ins/messages/message_types.h"

// TODO: Generate parameters using Python.
static const AioSourceToInsLabelMap kMap[kNumControllers] = {
  [kControllerA] = {
    .label = {
      [kInsMessageTypeInertial] = {
        [kAioNodeFcA] = kInsImuLabelPrimary,
      },
    },
    .valid = {
       [kInsMessageTypeInertial] = {
         [kAioNodeFcA] = true,
       },
     },
  },
  [kControllerB] = {
    .label = {
      [kInsMessageTypeInertial] = {
        [kAioNodeFcB] = kInsImuLabelPrimary,
      },
    },
    .valid = {
       [kInsMessageTypeInertial] = {
         [kAioNodeFcB] = true,
       },
     },
  },
  [kControllerC] = {
    .label = {
      [kInsMessageTypeInertial] = {
        [kAioNodeFcC] = kInsImuLabelPrimary,
      },
    },
    .valid = {
       [kInsMessageTypeInertial] = {
         [kAioNodeFcC] = true,
       },
     },
  },
};

const AioSourceToInsLabelMap *GetAioSourceToInsLabelMap(
    ControllerLabel controller) {
  assert(0 <= controller && controller < kNumControllers);
  return &kMap[controller];
}
