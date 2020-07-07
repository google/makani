%module physics
%{
#include <assert.h>

#include <vector>

#include "common/c_math/force_moment.h"
#include "common/c_math/linalg.h"
#include "common/c_math/vec3.h"
#include "common/runfiles_dir.h"
#include "control/system_params.h"
#include "sim/physics/aero.h"
#include "sim/physics/aero_frame.h"
#include "sim/physics/avl_aero_database.h"
#include "sim/physics/dvl_aero_database.h"
#include "sim/physics/motors.h"
#include "sim/physics/rotor_database.h"
#include "sim/physics/rotor_database_3d.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"
%}
%include stdint.i
%include std_string.i
%include typemaps.i
%include "common/c_math/force_moment.h"
%include "common/c_math/linalg.h"
%include "common/c_math/vec3.h"
%include "control/system_params.h"
%include "sim/physics/aero.h"
%include "sim/physics/aero_frame.h"
%include "sim/physics/avl_aero_database.h"
%include "sim/physics/dvl_aero_database.h"
%include "sim/physics/rotor_database.h"
%include "sim/physics/rotor_database_3d.h"
%include "sim/sim_params.h"
%include "sim/sim_types.h"

%inline %{

// Wrapper class around Vec to avoid explicit memory management.
class VecWrapper {
 public:
  VecWrapper(int32_t l) : vector_(VEC_MAX_ELEMENTS) {
    assert(0 <= l && l < VEC_MAX_ELEMENTS);
    vec_.length = l;
    vec_.d = vector_.data();
    vec_.max_length = VEC_MAX_ELEMENTS;
    vec_.variable_len = true;
  }

  void SetValue(int32_t i, double x) { *VecPtr(&vec_, i) = x; }
  const Vec &GetVec() const { return vec_; }

 private:
  std::vector<double> vector_;
  Vec vec_;
};

std::string GetRotorDatabase(const std::string &rel_path) {
  return RunfilesDir() + "/database/" + rel_path;
}

const AeroSimParams &GetAeroSimParams() {
  return GetSimParams()->aero_sim;
}

void OverrideCL0(double new_value) {
  GetSimParamsUnsafe()->aero_sim.coeff_offsets.CL = new_value;
}

%}

%apply double *OUTPUT {double *torque_lower_limit};
%apply double *OUTPUT {SimMotorLimit *lower_constraint};
%apply double *OUTPUT {double *torque_upper_limit};
%apply double *OUTPUT {SimMotorLimit *upper_constraint};
%inline %{

void CalcTorqueLimits(
    double voltage, double rotor_vel, const MotorParams &params,
    double *torque_lower_limit, SimMotorLimit *lower_constraint,
    double *torque_upper_limit, SimMotorLimit *upper_constraint) {
  ::sim::physics::motors::TorqueLimits limits
      = ::sim::physics::motors::CalcTorqueLimits(voltage, rotor_vel, params);
  *torque_lower_limit = limits.lower_limit;
  *lower_constraint = limits.lower_constraint;
  *torque_upper_limit = limits.upper_limit;
  *upper_constraint = limits.upper_constraint;
}

%}
