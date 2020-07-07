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

#include "sim/sim_save_states.h"

#include <stdint.h>

#include <string>

#include "sim/json_formatter.h"
#include "sim/models/model.h"

int32_t SaveStatesToFile(const std::string &filename, const Model *model,
                         uint64_t sim_time_step) {
  JsonFormatter out;
  out.Set("time", sim_time_step);
  if (model->Save(&out)) return -1;
  return out.WriteFile(filename);
}

int32_t LoadStatesFromFile(const std::string &filename, Model *model,
                           uint64_t *sim_time_step) {
  JsonFormatter in(filename);
  if (in.Get("time", sim_time_step)) return -1;
  if (model->Load(in)) return -1;
  return 0;
}
