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

#include "sim/faults/faults.h"

#include <glog/logging.h>

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "sim/sim_types.h"

FaultSchedule::FaultSchedule(const SimFaultParams *params)
    : fault_map_(), claimed_faults_() {
  if (params != nullptr) {
    CHECK_LE(0, params->num_fault_events);
    CHECK_GE(MAX_FAULT_EVENTS, params->num_fault_events);

    for (int32_t i = 0; i < params->num_fault_events; ++i) {
      AddFault(params->fault_events[i]);
    }
  }
}

FaultSchedule::FaultFunc FaultSchedule::ClaimFaultFunc(
    const std::string &component,
    const std::vector<std::pair<SimFaultType, uint32_t>> &fault_specs) {
  for (const auto &type_and_num_params : fault_specs) {
    FaultKey key(component, type_and_num_params.first);
    CHECK_EQ(claimed_faults_.count(key), 0U)
        << "when claiming faults for " << component << ". "
        << "Check that component names are unique.";
    claimed_faults_[key] = type_and_num_params.second;
  }

  return ([this, component](double t, SimFaultType type,
                            std::vector<double> *parameters) -> bool {
    return this->HasFault(t, component, type, parameters);
  });
}

void FaultSchedule::AddFault(const SimFaultEvent &event) {
  CHECK_LT(event.t_start, event.t_end);
  CHECK_LE(0, event.num_parameters);
  CHECK_GE(MAX_FAULT_EVENT_PARAMETERS, event.num_parameters);
  CHECK_GE(MAX_COMPONENT_NAME_LENGTH, strlen(event.component));

  FaultKey key(event.component, event.type);
  EventMap::iterator iter = fault_map_.find(key);

  std::vector<double> parameters(event.parameters,
                                 event.parameters + event.num_parameters);
  // This pointer will be managed by a std::unique_ptr below.
  std::unique_ptr<Event> fault(
      new Event(event.t_start, event.t_end, parameters));
  if (iter == fault_map_.end()) {
    std::unique_ptr<EventSet> fault_set(new EventSet());
    fault_set->insert(std::unique_ptr<Event>(fault.release()));
    fault_map_[key] = std::unique_ptr<EventSet>(fault_set.release());
  } else {
    std::pair<EventSet::iterator, bool> inserted =
        iter->second->insert(std::unique_ptr<Event>(fault.release()));
    CHECK(inserted.second) << "Two faults of the same type with overlapping"
                           << " time intervals were specified for the same"
                           << " component.";
  }
}

bool FaultSchedule::AllFaultsAreClaimed() const {
  bool all_correct = true;
  // Iterate over faults in the schedule.
  for (EventMap::const_iterator iter = fault_map_.begin();
       iter != fault_map_.end(); ++iter) {
    const FaultKey &key = iter->first;
    // Ensure the fault is actually simulated.
    FaultKeyParamCountMap::const_iterator found = claimed_faults_.find(key);
    if (found == claimed_faults_.end()) {
      all_correct = false;
      LOG(ERROR) << "Fault (" << key.component << ", "
                 << static_cast<int32_t>(key.type) << ") is not simulated.";
    } else {
      uint32_t num_parameters = found->second;
      // If the fault is simulated, check that every instance has the
      // correct number of parameters.
      const EventSet *event_set = iter->second.get();
      for (EventSet::const_iterator set_iter = event_set->begin();
           set_iter != event_set->end(); ++set_iter) {
        if (num_parameters != (*set_iter)->parameters.size()) {
          all_correct = false;
          LOG(ERROR) << "Fault (" << key.component << ", "
                     << static_cast<int32_t>(key.type) << ") wants "
                     << num_parameters << " parameters, found "
                     << (*set_iter)->parameters.size() << ".";
          // Break here as no more useful information is printed by this loop.
          break;
        }
      }
    }
  }
  return all_correct;
}

bool FaultSchedule::HasFault(double t, const std::string &component,
                             SimFaultType type,
                             std::vector<double> *parameters) const {
  FaultKey key(component, type);
  EventMap::const_iterator iter = fault_map_.find(key);
  // Return false if the component has no faults.
  if (iter == fault_map_.end()) return false;

  // Find the first element of this set which is of the correct
  // type and has t in [t_start, t_end).
  std::function<bool(const std::unique_ptr<Event> &)> search =
      [t, type](const std::unique_ptr<Event> &f) {
        return t >= f->t_start && t < f->t_end;
      };
  EventSet::iterator fault_iter =
      std::find_if(iter->second->begin(), iter->second->end(), search);
  if (fault_iter == iter->second->end()) {
    return false;
  } else {
    CHECK((*fault_iter)->parameters.size() == 0U || parameters != nullptr)
        << "A null parameters pointer was passed for a fault with "
        << (*fault_iter)->parameters.size() << " parameters.";
    if (parameters != nullptr) {
      *parameters = (*fault_iter)->parameters;
    }
    return true;
  }
}
