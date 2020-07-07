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

#include "control/actuator_util.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/linalg.h"
#include "common/c_math/optim.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/ground_frame.h"
#include "control/simple_aero.h"
#include "system/labels.h"
#include "system/labels_util.h"

bool ActuatorUtilValidateParams(const RotorControlParams *params) {
  assert(params != NULL);

  if (params->idle_speed < 0.0) {
    assert(!(bool)"idle_speed must be non-negative.");
    return false;
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    if (params->idle_speed >= params->max_speeds[i]) {
      assert(!(bool)"max_speeds must be greater than idle_speed.");
      return false;
    }
  }

  if (params->regularization_weight < 0.0) {
    assert(!(bool)"regularization_weight must be non-negative.");
    return false;
  }

  if (params->min_aero_power > -5e5) {
    assert(!(bool)"min_aero_power must be plausibly negative.");
    return false;
  }

  if (params->freestream_vel_table[0] < 0.0) {
    assert(!(bool)"freestream_vel_table must be non-negative.");
    return false;
  }

  for (int32_t j = 1; j < FREESTREAM_VEL_TABLE_LENGTH; ++j) {
    if (params->freestream_vel_table[j - 1] > params->freestream_vel_table[j]) {
      assert(!(bool)"freestream_vel_table must be strictly increasing.");
      return false;
    }
  }

  for (int32_t i = 0; i < 2; i++) {
    for (int32_t j = 0; j < kNumMotors; ++j) {
      for (int32_t k = 1; k < FREESTREAM_VEL_TABLE_LENGTH; ++k) {
        if (params->max_thrusts[i][j][k - 1] < params->max_thrusts[i][j][k]) {
          assert(
              !(bool)"max_thrusts should decrease with freestream velocity.");
          return false;
        }
      }
    }
  }

  if (params->total_power_limit_thrusts[0] < 0.0) {
    assert(!(bool)"total_power_limit_thrusts should be positive in still air.");
    return false;
  }
  for (int32_t j = 1; j < FREESTREAM_VEL_TABLE_LENGTH; ++j) {
    if (params->total_power_limit_thrusts[j - 1] <
        params->total_power_limit_thrusts[j]) {
      assert(!(bool)"total_power_limit_thrusts should decrease with velocity.");
      return false;
    }
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    if (params->simple_models[i].J_max < 0.0) {
      assert(!(bool)"J_max must be non-negative.");
      return false;
    }
    if (params->simple_models[i].J_neutral > params->simple_models[i].J_max) {
      assert(!(bool)"J_neutral must be less than J_max.");
      return false;
    }
    if (params->simple_models[i].D < 0.0) {
      assert(!(bool)"D must be non-negative.");
      return false;
    }
  }

  return true;
}

// Computes the expected maximum thrust for a given apparent wind.
double CalcMaxTotalThrust(double v_app, const RotorControlParams *params) {
  assert(v_app >= 0.0 && params != NULL);
  return Interp1(params->freestream_vel_table,
                 params->total_power_limit_thrusts, FREESTREAM_VEL_TABLE_LENGTH,
                 v_app, kInterpOptionSaturate);
}

// Calculate the maximum thrust for a given rotor based on a speed
// limit and model of thrust limits as a function of freestream
// velocity.  Also accounts for pylon structural limits.
static double CalcMaxThrust(double freestream_vel, double max_speed,
                            double air_density,
                            const double freestream_vel_table[],
                            const double max_thrusts[],
                            const double motor_mount_thrust_limit,
                            const SimpleRotorModelParams *params) {
  return fmin(
      fmin(Interp1(freestream_vel_table, max_thrusts,
                   FREESTREAM_VEL_TABLE_LENGTH, freestream_vel,
                   kInterpOptionSaturate),
           OmegaToThrust(max_speed, freestream_vel, air_density, params)),
      motor_mount_thrust_limit);
}

// Builds the constraint matrix and limits for the
// ThrustMomentToThrusts function.  Also, returns a vector that is
// guaranteed to meet the constraints.
static void SetupThrustPowerConstraints(StackingState stacking_state,
                                        const Vec *min_thrusts,
                                        const Vec *max_thrusts,
                                        double total_power_limit_thrust,
                                        const RotorControlParams *params,
                                        Mat *C, Vec *lower, Vec *upper,
                                        Vec *comm_and_diff_thrusts) {
  assert(VecIsSize(min_thrusts, kNumMotors));
  assert(VecIsSize(max_thrusts, kNumMotors));

  double max_min_thrusts = -INFINITY;
  double min_max_thrusts = INFINITY;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    assert(VecGet(max_thrusts, i) > VecGet(min_thrusts, i));
    if (VecGet(min_thrusts, i) > max_min_thrusts) {
      max_min_thrusts = VecGet(min_thrusts, i);
    }
    if (VecGet(max_thrusts, i) < min_max_thrusts) {
      min_max_thrusts = VecGet(max_thrusts, i);
    }
  }
  if (max_min_thrusts >= min_max_thrusts) {
    max_min_thrusts = min_max_thrusts - 1.0;
  }

  // Initial guess for common- and differential-mode thrusts that is
  // guaranteed to meet the individual thrust constraints defined
  // below.
  VecZero(comm_and_diff_thrusts);
  *VecPtr(comm_and_diff_thrusts, 0) = (min_max_thrusts + max_min_thrusts) / 2.0;

  // Initialize the constraint matrix.
  MatInit(&params->constraint_matrix[stacking_state][0][0], kNumMotors + 1, 5,
          C);

  // Set-up lower and upper bounds on individual thrusts.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    *VecPtr(lower, i) = fmin(VecGet(min_thrusts, i), max_min_thrusts);
    *VecPtr(upper, i) = VecGet(max_thrusts, i);
  }

  // Set-up maximum common-mode thrust bounds.  To ensure that the
  // initial guess is within the constraints we set the total thrust
  // bound to be at least as large as one more than the initial guess.
  *VecPtr(lower, kNumMotors) = -INFINITY;
  *VecPtr(upper, kNumMotors) =
      fmax(VecGet(comm_and_diff_thrusts, 0) + 1.0, total_power_limit_thrust);
}

// Convert common- and differential-mode thrusts solution to the
// predicted available thrust-moment.
static void CalcAvailableThrustMoment(StackingState stacking_state,
                                      const Vec *comm_and_diff_thrusts,
                                      const RotorControlParams *params,
                                      ThrustMoment *available_thrust_moment) {
  assert(kStackingStateNormal <= stacking_state &&
         stacking_state < kNumStackingStates);
  MAT_CLONE(
      4, 5, comm_and_diff_thrusts_to_thrust_moment,
      &params->comm_and_diff_thrusts_to_thrust_moment[stacking_state][0][0]);
  VEC_INIT(4, available_thrust_moment_vec, {0});
  MatVecMult(&comm_and_diff_thrusts_to_thrust_moment, comm_and_diff_thrusts,
             &available_thrust_moment_vec);
  available_thrust_moment->thrust = VecGet(&available_thrust_moment_vec, 0);
  available_thrust_moment->moment.x = VecGet(&available_thrust_moment_vec, 1);
  available_thrust_moment->moment.y = VecGet(&available_thrust_moment_vec, 2);
  available_thrust_moment->moment.z = VecGet(&available_thrust_moment_vec, 3);
}

// Solves a constrained least squares problem to approximate the
// required motor thrusts to achieve a desired total thrust and
// moment.
//
// Stacking constraint:
//
// The stacked motor system consists of four blocks, each block
// being a pair of motors:
//
//    PTO    PTI    STI    STO            Bot. Top
//   ( o )  ( o )  ( o )  ( o )  Block 1: SBO  PTO----------+
//     |______|______|______|    Block 2: SBI  PTI \__ inner \__outer
//     |      |      |      |    Block 3: PBI  STI /         /
//   ( o )  ( o )  ( o )  ( o )  Block 4: PBO  STO----------+
//    PBO    PBI    SBI    SBO
//
// Equal current must flow through each block, and the stacking motor
// controller attempts to regulate equal voltage on each block.  As a
// result, each block should be commanded to have equal power.  This
// function approximates this constraint by requesting thrusts such
// that the sum of the motor thrusts is the same for each block.
// During hover, when advance ratios are nearly zero and thrusts are
// nearly equal, this approximation is reasonable.
//
// Torque and advance ratio constraints:
//
// Individual motors have a maximum torque constraint during hover and
// transition-in and a maximum advance ratio constraint, i.e. minimum
// rotor speed before stall, in crosswind.  These constraints may be
// converted to equivalent thrust values using a model of the rotors,
// and thus are modeled as minimum and maximum thrust constraints.
//
// Total power constraint:
//
// The total power available to the rotors is, in general, less than
// the power that would be used if each rotor were at its maximum
// torque limit.  To model this, we pass in parameters for the power
// per unit thrust used by each rotor at the operating point, and use
// this to calculate a total power which must be greater than a given
// minimum (recall that negative power is thrusting).
//
// The decision variables for this optimization problem are:
//
//   x = [common_mode_thrust ; diff_thrust_1 ; ... ; diff_thrust_4]
//
// We minimize ||W*(Ax - b)||^2 where A and b have the following structure:
//
//   If penalizing the symmetric torsion mode:
//     A * x = [diff_thrust_1 ; diff_thrust_2 ; diff_thrust_3 ; diff_thrust_4 ;
//              thrust ; roll_moment ; pitch_moment ; yaw_moment;
//              symmetric_torsion_mode],
//     b = [0 ; 0 ; 0 ; 0 ;
//          thrust_cmd; roll_moment_cmd ; pitch_moment_cmd ; yaw_moment_cmd;
//          0],
//   Otherwise:
//     A * x = [diff_thrust_1 ; diff_thrust_2 ; diff_thrust_3 ; diff_thrust_4 ;
//              thrust ; roll_moment ; pitch_moment ; yaw_moment],
//     b = [0 ; 0 ; 0 ; 0 ;
//          thrust_cmd; roll_moment_cmd ; pitch_moment_cmd ; yaw_moment_cmd],
//
// and W is a diagonal matrix containing the associated weights.  The
// first four entries of these matrices are used to "regularize" the
// problem, ensuring that the cost function is strongly convex and that
// minimizers are unique.  This slightly biases the resulting total
// thrust and moment solutions, even when constraints are not hit.
// The objective of this regularization is to reduce the sensitivity
// of the solutions to small variations in problem inputs.
//
// Args:
//   thrust_moment: Desired thrust-moment command ([N] for thrust and
//       [N-m] for moment).
//   weights: Weight to place on each component ([1/N^2] for thrust and
//       [1/(N^2-m^2)] for moment).
//   min_thrusts: Vector of kNumMotors minimum thrusts.
//   max_thrusts: Vector of kNumMotors maximum thrusts.  Note that to
//       guarantee a valid initial guess for the constrained least
//       squares algorithm, we enforce that the maximum of the
//       min_thrusts is strictly less than the minimum of the
//       max_thrusts.
//   total_power_limit_thrust: Maximum total thrust command [N].  This is set to
//       roughly enforce a total power limit.
//   stacking_state: State of the stacked motor system.
//   params: Set of parameters describing the constraint matrix and
//       the conversions between common- and differential-thrusts and
//       individual thrusts.
//   thrusts: Output vector of kNumMotors thrusts.
//   available_thrust_moment: Output total thrust and moments
//       generated by thrusts.
static void ThrustMomentToThrusts(
    const ThrustMoment *thrust_moment, const ThrustMoment *weights,
    const Vec *min_thrusts, const Vec *max_thrusts,
    double total_power_limit_thrust, StackingState stacking_state,
    const RotorControlParams *params, Vec *thrusts,
    ThrustMoment *available_thrust_moment) {
  assert(kStackingStateNormal <= stacking_state &&
         stacking_state < kNumStackingStates);

  MAT_INIT(kNumMotors + 1, 5, C, {{0}});
  VEC_INIT(kNumMotors + 1, lower, {0});
  VEC_INIT(kNumMotors + 1, upper, {0});
  VEC_INIT(5, comm_and_diff_thrusts, {0});
  SetupThrustPowerConstraints(stacking_state, min_thrusts, max_thrusts,
                              total_power_limit_thrust, params, &C, &lower,
                              &upper, &comm_and_diff_thrusts);

  MAT_INIT(9, 5, A, {{0}});

  // Use the first four rows to select the differential thrusts.
  for (int32_t i = 0; i < 4; ++i) *MatPtr(&A, i, i + 1) = 1.0;

  // The next four rows calculate the thrust and moments.
  MAT_CLONE(
      4, 5, comm_and_diff_thrusts_to_thrust_moment,
      &params->comm_and_diff_thrusts_to_thrust_moment[stacking_state][0][0]);
  MatSubmatSet(&comm_and_diff_thrusts_to_thrust_moment, 0, 0, 4, 5, 4, 0, &A);

  // Use last row to select the symmetric torsion null space mode.
  *MatPtr(&A, 8, 1) = 1.0;
  *MatPtr(&A, 8, 2) = -1.0;
  *MatPtr(&A, 8, 3) = -1.0;
  *MatPtr(&A, 8, 4) = 1.0;

  // See the comment above for the structure of b.
  VEC_INIT(9, b,
           {0.0, 0.0, 0.0, 0.0, thrust_moment->thrust, thrust_moment->moment.x,
            thrust_moment->moment.y, thrust_moment->moment.z, 0.0});

  // See the comment above for the structure of weights_vec.
  VEC_INIT(9, weights_vec,
           {params->regularization_weight, params->regularization_weight,
            params->regularization_weight, params->regularization_weight,
            weights->thrust, weights->moment.x, weights->moment.y,
            weights->moment.z, params->symmetric_torsion_weight});

  // If not penalizing the symmetric torsion mode, only the first 8 rows of the
  // system should be used.
  if (!params->penalize_symmetric_torsion_mode) {
    A.nr = 8;
    b.length = 8;
    weights_vec.length = 8;
  }

  WeightLeastSquaresInputs(&weights_vec, &A, &b);
  ConstrainedLeastSquares(&A, &b, &C, &lower, &upper, &comm_and_diff_thrusts,
                          &comm_and_diff_thrusts);

  // Convert common- and differential-mode thrusts solution to
  // individual thrusts.
  MAT_CLONE(kNumMotors, 5, comm_and_diff_thrusts_to_thrusts,
            params->comm_and_diff_thrusts_to_thrusts);
  MatVecMult(&comm_and_diff_thrusts_to_thrusts, &comm_and_diff_thrusts,
             thrusts);

  CalcAvailableThrustMoment(stacking_state, &comm_and_diff_thrusts, params,
                            available_thrust_moment);
}

void MixRotors(const ThrustMoment *thrust_moment, const ThrustMoment *weights,
               double v_app, const Vec3 *pqr, StackingState stacking_state,
               bool force_zero_advance_ratio, double air_density,
               const RotorParams *const rotor_params[],
               const RotorControlParams *params, double *rotors,
               ThrustMoment *available_thrust_moment, double *v_app_locals) {
  // Determine thrust limits.
  VEC_INIT(kNumMotors, min_thrusts, {0});
  VEC_INIT(kNumMotors, max_thrusts, {0});
  for (int32_t i = 0; i < kNumMotors; ++i) {
    double v_app_local, min_thrust;
    if (force_zero_advance_ratio) {
      v_app_local = 0.0;
      min_thrust = 0.0;
    } else {
      v_app_local =
          CalcLocalAirspeed(v_app, rotor_params[i]->local_pressure_coeff,
                            &rotor_params[i]->pos, pqr);

      min_thrust = CalcMinThrust(v_app_local, params->idle_speed, air_density,
                                 &params->simple_models[i]);
    }

    // Store the estimated local apparent wind for telemetry
    v_app_locals[i] = v_app_local;

    int32_t max_thrust_params_index = stacking_state != kStackingStateNormal;
    double max_thrust =
        CalcMaxThrust(v_app_local, params->max_speeds[i], air_density,
                      params->freestream_vel_table,
                      params->max_thrusts[max_thrust_params_index][i],
                      params->motor_mount_thrust_limit[max_thrust_params_index],
                      &params->simple_models[i]);

    // Coerce min_thrust to be strictly less than max_thrust, which is
    // required by ThrustMomentToThrusts.
    if (min_thrust >= max_thrust) min_thrust = max_thrust - 1.0;
    *VecPtr(&min_thrusts, i) = min_thrust;
    *VecPtr(&max_thrusts, i) = max_thrust;
  }

  // Calculate the maximum common mode thrust.  We enforce a limit on
  // common mode thrust to roughly bound the total system power draw.
  double total_power_limit_thrust = CalcMaxTotalThrust(v_app, params);
  VEC_INIT(kNumMotors, thrusts, {0});
  ThrustMomentToThrusts(thrust_moment, weights, &min_thrusts, &max_thrusts,
                        total_power_limit_thrust, stacking_state, params,
                        &thrusts, available_thrust_moment);

  // Convert thrusts to rotor speeds using the simple propeller
  // models.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    double v_app_local =
        CalcLocalAirspeed(v_app, rotor_params[i]->local_pressure_coeff,
                          &rotor_params[i]->pos, pqr);
    rotors[i] = fmax(ThrustToOmega(VecGet(&thrusts, i), v_app_local,
                                   air_density, &params->simple_models[i]),
                     params->idle_speed);
  }
}

void MixFlaps(const Deltas *deltas, const double *offsets,
              const double *lower_limits, const double *upper_limits,
              double *flaps, Deltas *deltas_available) {
  assert(deltas != NULL);
  assert(offsets != NULL);
  assert(lower_limits != NULL);
  assert(upper_limits != NULL);
  assert(flaps != NULL);

  // MixFlaps is implemented as a constrained least squares problem.
  //
  // The decision variables for this optimization problem are:
  //
  //   x = [dA1 dA2 dA4 dA5 dA7 dA8 dE dR]
  //
  // Where dA1 (for example) represents the deflection of aileron A1
  // from its trim position, which is given in the "offsets" input.
  //
  // The constrained least squares solver finds x to minimize
  //
  //    ||W*(Ax - b)||^2
  //
  // subject to the constraint lower < Cx < upper, where W is a matrix
  // (typically diagonal) of weights.
  //
  // Rudder and elevator deflections are a simple pass-through. There
  // is no real need to use the quadratic program solver for them.
  // However, they are included here for generality, with the idea
  // that MixFlaps and MixRotors will eventually be combined.

  // The vector b includes a column of kNumFlaps zeros that are used
  // to implement the regularization condition, followed by the
  // incoming delta commands.
  VEC_INIT(kNumFlaps + NUM_DELTAS, b,
           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // Used for regularization.
            deltas->aileron, deltas->inboard_flap, deltas->midboard_flap,
            deltas->outboard_flap, deltas->elevator, deltas->rudder});

  // The first kNumFlaps = 8 rows of A consist of an identity matrix
  // selecting the individual control surfaces deflections. This is
  // used to provide solution regularization (all else being equal,
  // minimize the sum of squares of individual deflections).
  //
  // The next block of 6 rows extracts the aileron, flap, rudder, and
  // elevator deltas. The first row, computing aileron delta, is
  // perhaps the most interesting. The normalization is chosen so
  // that [-1, -1, +1, +1] generates one unit of delta, to match the
  // previous implementation.
  MAT_INIT(kNumFlaps + NUM_DELTAS, kNumFlaps, A,
           {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},    // A1
            {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},    // A2
            {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},    // A4
            {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0},    // A5
            {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},    // A7
            {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},    // A8
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},    // Elevator
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},    // Rudder
            {-1.0 / 4.0, -1.0 / 4.0, 0.0,                // ------------
             0.0, 1.0 / 4.0, 1.0 / 4.0, 0.0, 0.0},       // delta aileron
            {0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0},    // inboard flap
            {0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0},    // midboard flap
            {0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0},    // outboard flap
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},    // delta elevator
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}});  // delta rudder

  // The C matrix, used to form inequality constraints, just extracts
  // the individual control sufrace deflections, so that we can apply
  // actuator limits.  It is an identity matrix.
  MAT_INIT(kNumFlaps, kNumFlaps, C,
           {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},    // A1
            {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},    // A2
            {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},    // A4
            {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0},    // A5
            {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},    // A7
            {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},    // A8
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},    // Elevator
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}});  // Rudder

  // Turn the absolute limits on flap positions into limits on deltas
  // from trim.
  VEC(kNumFlaps, lower);
  VEC(kNumFlaps, upper);
  for (int i = 0; i < kNumFlaps; ++i) {
    *VecPtr(&lower, i) = lower_limits[i] - offsets[i];
    *VecPtr(&upper, i) = upper_limits[i] - offsets[i];
  }

  // Vector with initial guess at solution. This must obey the
  // constraints. To assure feasibility, we average the upper and
  // lower limits.
  VEC(kNumFlaps, x);
  VecLinComb(0.5, &lower, 0.5, &upper, &x);

  // Vector of weights.  The regularization conditions have a very
  // small weight compared to the other rows.
  VEC_INIT(kNumFlaps + NUM_DELTAS, w, {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3,
                                       1e-3, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

  // Solve it!
  WeightLeastSquaresInputs(&w, &A, &b);
  ConstrainedLeastSquares(&A, &b, &C, &lower, &upper, &x, &x);

  // Add the trim position to get the final output.
  // Note that the final outputs may violate the limits by a tiny
  // amount (floating point epsilon, approx 1e-17 radians) due to
  // rounding error.  Because the commands are saturated downstream
  // this should not be an issue.
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    flaps[i] = offsets[i] + VecGet(&x, i);
  }

  // Calculate the deltas that were achieved.
  MatVecMult(&A, &x, &b);
  if (deltas_available != NULL) {
    deltas_available->aileron = VecGet(&b, kNumFlaps + 0);
    deltas_available->inboard_flap = VecGet(&b, kNumFlaps + 1);
    deltas_available->midboard_flap = VecGet(&b, kNumFlaps + 2);
    deltas_available->outboard_flap = VecGet(&b, kNumFlaps + 3);
    deltas_available->elevator = VecGet(&b, kNumFlaps + 4);
    deltas_available->rudder = VecGet(&b, kNumFlaps + 5);
  }
}

void ServoAnglesToFlapAngles(const double servo_angles[],
                             const ServoParams params[], double flap_angles[]) {
  flap_angles[kFlapA1] =
      servo_angles[kServoA1] * params[kServoA1].linear_servo_to_flap_ratio;
  flap_angles[kFlapA2] =
      servo_angles[kServoA2] * params[kServoA2].linear_servo_to_flap_ratio;
  flap_angles[kFlapA4] =
      servo_angles[kServoA4] * params[kServoA4].linear_servo_to_flap_ratio;
  flap_angles[kFlapA5] =
      servo_angles[kServoA5] * params[kServoA5].linear_servo_to_flap_ratio;
  flap_angles[kFlapA7] =
      servo_angles[kServoA7] * params[kServoA7].linear_servo_to_flap_ratio;
  flap_angles[kFlapA8] =
      servo_angles[kServoA8] * params[kServoA8].linear_servo_to_flap_ratio;
  flap_angles[kFlapEle] =
      0.5 *
      (servo_angles[kServoE1] * params[kServoE1].linear_servo_to_flap_ratio +
       servo_angles[kServoE2] * params[kServoE2].linear_servo_to_flap_ratio);
  flap_angles[kFlapRud] =
      0.5 *
      (servo_angles[kServoR1] * params[kServoR1].linear_servo_to_flap_ratio +
       sin(servo_angles[kServoR1]) *
           params[kServoR1].nonlinear_servo_to_flap_ratio +
       servo_angles[kServoR2] * params[kServoR2].linear_servo_to_flap_ratio +
       sin(servo_angles[kServoR2]) *
           params[kServoR2].nonlinear_servo_to_flap_ratio);
}

static double FlapAngleToServoAngle(double flap_angle,
                                    const ServoParams *params) {
  assert(params->linear_servo_to_flap_ratio == 0.0 ||
         params->nonlinear_servo_to_flap_ratio == 0.0);
  assert(params->linear_servo_to_flap_ratio != 0.0 ||
         params->nonlinear_servo_to_flap_ratio != 0.0);
  if (params->linear_servo_to_flap_ratio != 0.0) {
    return flap_angle / params->linear_servo_to_flap_ratio;
  } else {
    return Asin(flap_angle / params->nonlinear_servo_to_flap_ratio);
  }
}

void FlapAnglesToServoAngles(const double flap_angles[],
                             const ServoParams params[],
                             double servo_angles[]) {
  for (int32_t i = 0; i < kNumServos; ++i) {
    servo_angles[i] =
        FlapAngleToServoAngle(flap_angles[ServoToFlap(i)], &params[i]);
  }
}

double CalcHoverGsTargetAzimuthReel(const Vec3 *vessel_pos_g,
                                    const Mat3 *dcm_g2v, const Vec3 *wing_pos_g,
                                    const Gs02Params *params,
                                    bool *target_valid) {
  // Calculate the displacement vector from the platform/vessel origin to the
  // wing origin.  TODO: We may wish to directly subtract the GPS
  // measurements from the base station and the rover (taking the Kalman filters
  // out of the picture) to achieve better common mode noise supression.
  Vec3 gs_to_wing_g;
  Vec3Sub(wing_pos_g, vessel_pos_g, &gs_to_wing_g);

  // TODO: I am worried about possible instabilities in the following
  // arising from the method in which dcm_g2v is calculated. As perch_azi is
  // used in the calculation of dcm_g2v, and here we are using dcm_g2v in
  // calculating the perch_azi command, there is the possibility for unwanted
  // dynamics.
  Vec3 gs_to_wing_v;
  Mat3Vec3Mult(dcm_g2v, &gs_to_wing_g, &gs_to_wing_v);

  double r_wing = Vec3XyNorm(&gs_to_wing_v);
  double r_anchor = params->anchor_arm_length;

  *target_valid = r_wing > r_anchor;

  // TODO: Account for ground station attitude (mainly
  // heading).
  double gs_azi_cmd =
      VecGToAzimuth(&gs_to_wing_v) + Asin(r_anchor / fmax(r_wing, r_anchor));

  // TODO: I expect that the azi command should be in in -PI, PI.
  return Wrap(gs_azi_cmd, 0.0, 2.0 * PI);
}
