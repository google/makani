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

#ifndef SIM_FAULTS_FAULTS_H_
#define SIM_FAULTS_FAULTS_H_

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "common/macros.h"
#include "sim/sim_types.h"

// This type uniquely identifies a fault as a combination of strings
// indicating the component being affected
// (e.g. "Loadcell/tensions[0]") and the type of fault.
struct FaultKey {
  FaultKey(const std::string &component__, SimFaultType type__)
      : component(component__), type(type__) {}
  FaultKey() : FaultKey("", kSimFaultNoFault) {}

  std::string component;
  SimFaultType type;
};

class CompareFaultKey {
 public:
  bool operator()(const FaultKey &k1, const FaultKey &k2) const {
    return (k1.component < k2.component) ||
           (k1.component == k2.component && k1.type < k2.type);
  }
};

typedef std::map<FaultKey, uint32_t, CompareFaultKey> FaultKeyParamCountMap;

// A FaultSchedule provides an interface for checking if a simulated fault
// of a given type is active for a specific component.  The class also tracks
// whether a fault was checked at any point.
//
// Every fault event is associated with a component (specified by the
// full name of a model) and a type (defined by SimFaultType).  Each
// event has a duration and parameter list.  Faults for the same
// (component, type) combination cannot be specified with overlapping
// time intervals.
class FaultSchedule {
 public:
  // A FaultFunc is a function called to determine if a fault of a
  // given type is active at time t (in which case true is returned).
  // If a fault is active and has parameters, then the parameters are
  // written out to the remaining arguments.parameters.
  //
  // See FaultSchedule::ClaimFaultFunc below.
  typedef std::function<bool(double t, SimFaultType type,
                             std::vector<double> *parameters)>
      FaultFunc;

  explicit FaultSchedule(const SimFaultParams *params);
  virtual ~FaultSchedule() {}

  // Returns the FaultFunc to determine if a fault is active for a
  // component, and claims it from the schedule. CHECK-fails if the fault
  // is already claimed.
  //
  // Args:
  //   component: The full name of a model or state.
  //
  // Returns:
  //   A FaultFunc that indicates if a fault is active and, if so,
  //   returns the associated parameters.
  FaultFunc ClaimFaultFunc(
      const std::string &component,
      const std::vector<std::pair<SimFaultType, uint32_t>> &fault_specs);

  // Adds a fault to a FaultSchedule defined on the interval
  // [event.t_start, event.t_end).  Note that the number of parameters
  // given must agree with the number requested by a model checking
  // for this fault.
  //
  // Args:
  //   event: A SimFaultEvent to be added.
  void AddFault(const SimFaultEvent &event);

  // Returns whether all faults in the schedule have been claimed.
  //
  // Args:
  //   sim_faults: A map from FaultKey to the number of parameters
  //       expected for a given key.
  //
  // Returns:
  //   True if all of the faults in the FaultSchedule have a key in
  //   sim_faults and the correct number of parameters.
  bool AllFaultsAreClaimed() const;

 protected:
  struct Event {
    double t_start;
    double t_end;
    std::vector<double> parameters;

    Event(double t_start__, double t_end__, std::vector<double> parameters__)
        : t_start(t_start__), t_end(t_end__), parameters(parameters__) {}
  };

  class CompareEvent {
   public:
    // This comparison is designed to avoid having events which are defined on
    // overlapping time intervals.
    bool operator()(const std::unique_ptr<Event> &e1,
                    const std::unique_ptr<Event> &e2) {
      return e1->t_end <= e2->t_start;
    }
  };

  // For each (component, type) we maintain a set of Event structures
  // sorted by the timing.
  typedef std::set<std::unique_ptr<Event>, CompareEvent> EventSet;
  typedef std::map<FaultKey, std::unique_ptr<EventSet>, CompareFaultKey>
      EventMap;

  // Determines if a given FaultKey has a fault.  If so, the
  // parameters are written into the given vector.
  bool HasFault(double t, const std::string &component, SimFaultType type,
                std::vector<double> *parameters) const;

  // The overall data of a schedule is a map from (component, type) to sets.
  EventMap fault_map_;

  // Tracks which faults have been claimed.
  FaultKeyParamCountMap claimed_faults_;

 private:
  friend class FaultScheduleTest;

  DISALLOW_COPY_AND_ASSIGN(FaultSchedule);
};

#endif  // SIM_FAULTS_FAULTS_H_
