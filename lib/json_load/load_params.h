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

// Interface to all the parameters that may be loaded from JSON files.

#ifndef LIB_JSON_LOAD_LOAD_PARAMS_H_
#define LIB_JSON_LOAD_LOAD_PARAMS_H_

#include "control/control_types.h"
#include "control/system_types.h"
#include "gs/monitor/monitor_types.h"
#include "sim/sim_types.h"

namespace json_load {

void LoadControlParams(ControlParams *control_params);
void LoadSimParams(SimParams *sim_params);
void LoadSystemParams(SystemParams *system_params);
void LoadMonitorParams(MonitorParams *monitor_params);

}  // namespace json_load

#endif  // LIB_JSON_LOAD_LOAD_PARAMS_H_
