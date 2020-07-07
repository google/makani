// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lib/json_load/load_params.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <jansson.h>

#include <string>

#include "control/control_types.h"
#include "control/system_types.h"
#include "gs/monitor/monitor_types.h"
#include "lib/json_load/json_load.h"
#include "lib/json_load/json_load_or_die.h"
#include "lib/json_load/json_load_basic.h"
#include "sim/sim_types.h"

DEFINE_string(all_params, "",
              "Path to JSON file containing all alternative parameters.");
DEFINE_string(control_params, "",
              "Path to JSON file containing alternative control parameters.");
DEFINE_string(monitor_params, "",
              "Path to JSON file containing alternative monitor parameters.");
DEFINE_string(sim_params, "",
              "Path to JSON file containing alternative simulator parameters.");
DEFINE_string(system_params, "",
              "Path to JSON file containing alternative system parameters.");

namespace json_load {

namespace {

// Loads a JSON file and either grabs the sub-dictionary specified by
// param_name if the all_params flag was used, or otherwise, grabs the
// entire dictionary.  Returns a Jansson object pointer to the
// parameter structure.
json_t *LoadParamsHelper(const std::string &param_name,
                         const std::string &filename) {
  if (!FLAGS_all_params.empty()) {
    CHECK(filename.empty()) << "Both all_params and " << param_name
                            << "_params files were specified.";

    json_t *f = LoadFileOrDie(FLAGS_all_params);
    if (f == nullptr) return nullptr;
    json_t *json_params = JSONLoadStructGet(f, param_name.c_str());

    // We're finished with f, but we need to ensure that json_params stays
    // alive.
    json_incref(json_params);
    json_decref(f);

    return json_params;
  } else if (!filename.empty()) {
    return LoadFileOrDie(filename);
  }

  return nullptr;
}

}  // namespace

// The following four functions load the specified structure
// (ControlParams, SimParams, SystemParams, or MonitorParams) from the
// file specified in the flags.  If no file is specified, the
// parameter structure is unchanged.
void LoadControlParams(ControlParams *control_params) {
  json_t *json_params = LoadParamsHelper("control", FLAGS_control_params);
  if (json_params == nullptr) return;
  CHECK_EQ(0, JSONLoadStruct_ControlParams(json_params, control_params))
      << "Failed to load control parameters.";
  json_decref(json_params);
}

void LoadSimParams(SimParams *sim_params) {
  json_t *json_params = LoadParamsHelper("sim", FLAGS_sim_params);
  if (json_params == nullptr) return;
  CHECK_EQ(0, JSONLoadStruct_SimParams(json_params, sim_params))
      << "Failed to load sim parameters.";
  json_decref(json_params);
}

void LoadSystemParams(SystemParams *system_params) {
  json_t *json_params = LoadParamsHelper("system", FLAGS_system_params);
  if (json_params == nullptr) return;
  CHECK_EQ(0, JSONLoadStruct_SystemParams(json_params, system_params))
      << "Failed to load system parameters.";
  json_decref(json_params);
}

void LoadMonitorParams(MonitorParams *monitor_params) {
  json_t *json_params = LoadParamsHelper("monitor", FLAGS_monitor_params);
  if (json_params == nullptr) return;
  CHECK_EQ(0, JSONLoadStruct_MonitorParams(json_params, monitor_params))
      << "Failed to load monitor parameters.";
  json_decref(json_params);
}

}  // namespace json_load
