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

// The Model class represents any object that has continuous,
// discrete, or other forms of state (similar to a block in a Simulink
// diagram).  It is responsible for:
// - Transferring continuous state to the ODE solver.
// - Storing the previous step time (t_z1) for discrete state.
// - Updating derived states from the continuous or discrete states.
// - Tracking which faults a model can simulate.
// - Saving and loading state information from file.

#ifndef SIM_MODELS_MODEL_H_
#define SIM_MODELS_MODEL_H_

#include <float.h>
#include <glog/logging.h>
#include <stdint.h>

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "sim/faults/faults.h"
#include "sim/interfaces.h"
#include "sim/sim_types.h"
#include "sim/state.h"

// Connections control the flow of information between models and are also used
// within a model to update derived states. A connection function may be as
// simple as passing the output of a getter from one model to the setter of
// another.
//
// Most connections are not time-dependent, but in some cases they are, e.g. for
// disturbance injection and log replay.
typedef std::function<void(double)> Connection;

// A ConnectionStore manages dependencies between connections through update
// levels. Connections are executed one level at a time, starting at level 0. If
// connection A must be called before connection B, then A's level should be
// less than B's level.
class ConnectionStore {
 public:
  ConnectionStore() : connections_() {}

  void Add(int32_t level, Connection connection) {
    if (connections_.size() < static_cast<size_t>(level + 1)) {
      connections_.resize(level + 1);
    }
    connections_[level].push_back(connection);
  }

  void Run(double t) {
    for (const auto &connection_vector : connections_) {
      for (const Connection &connection : connection_vector) {
        connection(t);
      }
    }
  }

 private:
  std::vector<std::vector<Connection>> connections_;
};

class Model : virtual public PublishableInterface,
              virtual public DictLoadableInterface {
 public:
  Model(const std::string &parent_full_name, const std::string &name__,
        double ts)
      : ts_(ts),
        connections_(),
        sub_models_(),
        name_(name__),
        full_name_(JoinName(parent_full_name, name__)),
        input_values_(),
        continuous_states_(),
        discrete_states_(),
        derived_values_(),
        t_z1_(-DBL_MAX),
        num_states_(0),
        setup_done_(false) {}
  Model(const std::string &parent_full_name, const std::string &name__)
      : Model(parent_full_name, name__, 0.0) {}
  Model(const std::string &name__, double ts) : Model("", name__, ts) {}
  explicit Model(const std::string &name__) : Model(name__, 0.0) {}
  virtual ~Model() __attribute__((noinline)) {}

  const std::string &name() const { return name_; }
  const std::string &full_name() const { return full_name_; }

  int32_t num_states() const {
    DCHECK(setup_done_) << "Model " << name_ << ": SetupDone() must be called "
                        << "prior to num_states().";
    return num_states_;
  }

  const std::vector<ContinuousStateInterface *> &continuous_states() const {
    return continuous_states_;
  }

  const std::vector<StateInterface *> &input_values() const {
    return input_values_;
  }

  const std::vector<StateInterface *> &derived_values() const {
    return derived_values_;
  }

  const std::vector<Model *> sub_models() const { return sub_models_; }

  void SetStateFromVector(const double state_vec[]);
  void GetStateToVector(double state_vec[]) const;
  void GetDerivToVector(double deriv_vec[]) const;

  virtual int32_t Save(DictFormatterInterface *out) const;
  virtual int32_t Load(const DictFormatterInterface &in);

  // Clears all state related to integration -- ContinuousStates, input values,
  // and derived values.
  void ClearIntegrationState();

  void ClearInputValues();

  // Calculates the derivatives of all the continuous states in this
  // model and its sub-models.
  void CalcDeriv(double t);

  // Evaluates all connections, both between models and for calculating derived
  // states internal to a model.
  void RunConnections(double t) { connections_.Run(t); }

  // Performs the discrete time update of a model if sufficient time
  // has elapsed.  Calls DiscreteStepHelper if ts_ time has elapsed
  // since the last update (with some tolerance), then calls
  // DiscreteUpdate for each sub-model.
  //
  // Args:
  //   t: Current simulator time.
  void DiscreteUpdate(double t);

  // Displays text describing the current state of the class.  Used
  // for debugging purposes only.
  void Disp() const;

  virtual void Publish() const {}

 protected:
  // Recursively adds internal connections for all models.
  void AddAllInternalConnections(ConnectionStore *connections) {
    AddInternalConnections(connections);
    for (Model *sub_model : sub_models_) {
      sub_model->AddAllInternalConnections(connections);
    }
  }

  // Returns a reference to a new pointer at the end of the indicated state
  // list. Used to track states immediately upon construction.
  ContinuousStateInterface **new_continuous_state() {
    continuous_states_.push_back(nullptr);
    return &continuous_states_.back();
  }
  DiscreteStateInterface **new_discrete_state() {
    discrete_states_.push_back(nullptr);
    return &discrete_states_.back();
  }
  StateInterface **new_derived_value() {
    derived_values_.push_back(nullptr);
    return &derived_values_.back();
  }
  StateInterface **new_input_value() {
    input_values_.push_back(nullptr);
    return &input_values_.back();
  }

  static std::string JoinName(const std::string &parent_full_name,
                              const std::string &name__) {
    return parent_full_name.empty() ? name__ : parent_full_name + "/" + name__;
  }

  // A child class must call this once setup is complete.
  void SetupDone() {
    num_states_ = CalcNumStates();
    setup_done_ = true;
  }

  // Sets the list of models for which this model is the parent.
  // These models are automatically cleared, saved, loaded, and
  // displayed when the same operation is applied to the parent.
  // UpdateClassFromStates, CalcDeriv, and DiscreteUpdate automatically
  // update sub-modules after the associated helper for the parent
  // class has been called.
  void set_sub_models(const std::vector<Model *> &v) {
    DCHECK(!setup_done_) << "Model " << name_ << ": set_sub_models() may not "
                         << "be called after SetupDone().";
    sub_models_ = v;
  }

  void add_sub_model(Model *m) { sub_models_.push_back(m); }

  // Registers a particular fault type as being simulated by this model.
  //
  // Args:
  //   sub_name: A string indicating a subsystem to which a fault applies or
  //       an empty string if the fault applies directly to this model.  The
  //       subsystem need not be a sub-model (e.g. the name could indicate
  //       a state).
  //   type: The fault type.
  //   num_parameters: A non-negative integer specifying how many parameters
  //       are required for simulating this fault.

  void ClearContinuousStates();
  void ClearDiscreteStates();
  void ClearDerivedValues();

  double t_z1() const { return t_z1_; }

  // Sample time for the discrete update step.
  const double ts_;

  // Defines connections between sub-models.
  ConnectionStore connections_;

  // Models can contain sub-models which hold their own state and
  // connections.
  std::vector<Model *> sub_models_;

 private:
  // Calculates num_states_.  num_states() is called frequently, and
  // precomputing has a significant impact on performance.
  int32_t CalcNumStates();

  // Adds a model's internal connections to the ConnectionStore. These are
  // typically used to update derived states.
  virtual void AddInternalConnections(ConnectionStore * /*connections*/) {}

  // Updates the derivatives of the continuous states.  In a sense,
  // this is the last "level" of the UpdateClassFromStates function
  // since it must be called after all of the derived values in all
  // the models are updated.  However, it is distinguished from
  // UpdateClassFromStates by only acting on the derivatives of
  // continuous states and accepting a time input argument (t).
  virtual void CalcDerivHelper(double /*t*/) {
    CHECK(continuous_states_.empty());
  }

  // DiscreteStepHelper is called whenever the elapsed time exceeds the
  // sample time for the model (see DiscreteUpdate).  The model should
  // update all of its discrete states in its implementation of this
  // function.
  virtual void DiscreteStepHelper(double t) = 0;

  // Name of the current model, not including the name of any parent
  // models.
  const std::string name_;

  // Full name of the current model and all parent models in the
  // format: parent_full_name/model_name.
  const std::string full_name_;

  // Pointers to the values, which are used as inputs to the model.
  // The StateInterface is used to keep track of when these values
  // have been updated.
  std::vector<StateInterface *> input_values_;

  // Pointers to the continuous states of the model, i.e. states that
  // are passed to the ODE solver.
  std::vector<ContinuousStateInterface *> continuous_states_;

  // Pointers to the discrete states of the systems.  These states are
  // updated at a fixed sample rate set by ts_.
  std::vector<DiscreteStateInterface *> discrete_states_;

  // Pointers to derived values, which depend on the continuous and
  // discrete states of the system.  These are both used to cache
  // intermediate values used in calculations, and also as the outputs
  // of the model.  The StateInterface is used to keep track of when
  // these values have been updated.
  std::vector<StateInterface *> derived_values_;

  // Last time [s] that the discrete states were updated.
  double t_z1_;

  // Number of states in the current model and all its sub-models.
  int32_t num_states_;

  // Indicates that we are done initializing the model.
  bool setup_done_;
};

#endif  // SIM_MODELS_MODEL_H_
