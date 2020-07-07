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

#ifndef SIM_MODELS_BASE_SYSTEM_MODEL_H_
#define SIM_MODELS_BASE_SYSTEM_MODEL_H_

#include <memory>
#include <string>
#include <vector>

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/model.h"
#include "sim/models/sensors/sensor.h"
#include "sim/sim_messages.h"

class BaseSystemModel : public Model {
 public:
  BaseSystemModel(const std::string &parent_full_name,
                  const SystemParams &system_params,
                  const SimParams &sim_params, FaultSchedule *faults);
  ~BaseSystemModel() {}

  void Publish() const override;

  const std::vector<std::unique_ptr<Sensor>> &sensors() const {
    return sensors_;
  }

 protected:
  // Parameters.
  const SystemParams &system_params_;
  const SimParams &sim_params_;
  FaultSchedule *faults_;

  // Sub-models.
  std::vector<std::unique_ptr<Sensor>> sensors_;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseSystemModel);
};

#endif  // SIM_MODELS_BASE_SYSTEM_MODEL_H_
