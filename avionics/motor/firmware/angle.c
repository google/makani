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

#include "avionics/motor/firmware/angle.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/fast_math/filter.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/motor/firmware/angle_meas.h"
#include "avionics/motor/firmware/calib_params.h"
#include "avionics/motor/firmware/params.h"
#include "common/macros.h"

#define ZERO_CROSSING_DT_MAX (UINT32_MAX / 2)
#define MAX_SENSOR_POLE_PAIRS 32

static float g_period;  // Update period [s].

// Precalculate the value 2*pi*f_clock.
static const float k2PiClockFreq = 2.0f * PI_F * CLOCK32_HZ;

typedef enum {
  kZeroCrossingNone,
  kZeroCrossingCosFall,
  kZeroCrossingSinFall,
  kZeroCrossingCosRise,
  kZeroCrossingSinRise,
} ZeroCrossingType;

typedef struct {
  // Expected direction of next crossings.
  ZeroCrossingType next_crossing;
  int32_t direction;

  // Last samples of the input signals.
  int32_t sin_last;
  int32_t cos_last;

  // Crossing and sample times.
  uint32_t dt32;          // Most recent time delta.
  uint32_t t32_last;      // Time of last sample.
  uint32_t t32_cos_fall;  // Time of last cos fall, etc.
  uint32_t t32_cos_rise;
  uint32_t t32_sin_fall;
  uint32_t t32_sin_rise;

  FirstOrderFilterState omega_filter;
} ZeroCrossingState;

typedef struct {
  float theta_m;            // Estimated mechanical angle.
  float theta_m_error_int;  // Integral of mechanical angle error.

  FirstOrderFilterState omega_filter;
} DerivState;

typedef struct {
  float val;
  float sum;
  float sum_z;
  float n;
  float n_z;

  float offset[2 * MAX_SENSOR_POLE_PAIRS];
  float scale[2 * MAX_SENSOR_POLE_PAIRS];
  int32_t i_z;
  int32_t i_zz;

  int32_t spin_dir_z;
  bool enable;
} SensorProfile;

typedef struct {
  SensorProfile sin;
  SensorProfile cos;

  // Thresholds for updating the bias and scale factors.
  float n_enable;
  float n_disable;
} SensorProfileData;

typedef struct {
  DerivState deriv;
  SensorProfileData sensor;
  ZeroCrossingState zc;
} MotorAngleState;

// cpplint, xemacs, and clang-format can't agree on how to format this.
// clang-format off
static MotorAngleState g_state = {
  .deriv = {
    .theta_m = 0.0f,
    .theta_m_error_int = 0.0f,
  },
  .sensor = {
    .sin = {.enable = false},
    .cos = {.enable = false},
  },
  .zc = {0},
};

MotorAngleParams g_motor_angle_params = {
  .deriv = {
    .theta_kp = 0.15f,
    .theta_ki = 112.2f,            // [s]
    .omega_filter_pole = -300.0f,  // [rad/s]
  },
  .zc = {
    .omega_filter_pole = -2.0f * PI_F * 30.0f,  // [rad/s]
  },
  .sensor = {
    .omega_enable_threshold = 25.0f,
    .omega_disable_threshold = 20.0f,
    .filter_pole = 0.8
  },
  .theta_offset_m = 0.0f,
  .omega_transition_deriv = 0.0f,  // [rad/s (mechanical)]
  .omega_transition_zc = 0.0f,     // [rad/s (mechanical)]
};
// clang-format on

static void SensorProfileReset(float period, const MotorParams *motor_params,
                               SensorProfileParams *params,
                               SensorProfileData *data) {
  float npps = motor_params->num_pole_pairs_sens;

  // Guard against division by zero as a result of parameter modification.
  params->omega_enable_threshold =
      Maxf(fabsf(params->omega_enable_threshold), 1.0f);
  params->omega_disable_threshold =
      Maxf(fabsf(params->omega_disable_threshold), 1.0f);

  // Guard against division by zero in the update algorithm.
  data->n_enable =
      Maxf(PI_F / (npps * params->omega_enable_threshold * period), 1.0f);
  data->n_disable =
      Maxf(PI_F / (npps * params->omega_disable_threshold * period), 1.0f);
}

static void SensorProfileInit(float period, const MotorAngleMeas *meas,
                              const MotorParams *motor_params,
                              SensorProfileParams *params,
                              SensorProfileData *data) {
  float npps = motor_params->num_pole_pairs_sens;
  assert(npps <= MAX_SENSOR_POLE_PAIRS);

  float mag = sqrtf(meas->cos_m * meas->cos_m + meas->sin_m * meas->sin_m);
  for (int32_t i = 0; i < 2 * (int32_t)npps; ++i) {
    data->sin.offset[i] = 0;
    data->cos.offset[i] = 0;
    data->sin.scale[i] = mag;
    data->cos.scale[i] = mag;
  }
  data->sin.val = meas->sin_m;
  data->cos.val = meas->cos_m;

  SensorProfileReset(period, motor_params, params, data);
}

// Wrap the float x into the interval [0, x_lim) assuming x starts in the
// interval [-x_lim, 2*x_lim).
static inline float Wrapf(float x, float x_lim) {
  if (x < 0.0f) x += x_lim;    // This order correctly enforces [0, x_lim) when,
  if (x >= x_lim) x -= x_lim;  // e.g. -FLT_EPSILON * x_lim < x < 0.
  return x;
}

static inline void SensorProfileInterp(float xi, float npps,
                                       const SensorProfile *data, float *val) {
  float len = 2.0f * npps;

  // xi may be outside of [0, len) due to the rotation or the offset for cos.
  xi = Wrapf(xi, len);

  // Find the indices and evaluate the shape functions for linear interpolation.
  // Note that the bound on xi at 2*npps is exclusive which prevents array
  // overflow with i1 and guarantees xi2 is non-negative.
  int32_t i1 = (int32_t)xi;
  int32_t i2 = i1 + 1 >= len ? 0 : i1 + 1;
  float xi2 = xi - i1;
  float xi1 = 1.0f - xi2;

  // Perform the interpolation and apply the correction.
  float offset = xi1 * data->offset[i1] + xi2 * data->offset[i2];
  float scale = xi1 * data->scale[i1] + xi2 * data->scale[i2];
  *val = (*val - offset) / Maxf(scale, 1.0f);  // Guard against divide by zero.
}

// Correct the raw sin and cos signals using the sensor bias profiles.
//
// The mechanical angle is used to index into the offset and scale arrays as
// shown below for a 3 pole pair sensor. The cos and sin signals have nodes for
// for interpolation at their respective zero crossings which results in the
// theta_s = pi/2 rad offset. The intermediate interpolation variable xi is also
// shown for reference.
//
//   sin        |---------|---------|---------|---------|---------|---------|
//   index      0         1         2         3         4         5         0
//
//   cos        -----|---------|---------|---------|---------|---------|-----
//   index           0         1         2         3         4         5
//
//              +----+----+----+----+----+----+----+----+----+----+----+----+
//   theta_s:   1  -1/2   0   1/2   1  -1/2   0   1/2   1  -1/2   0   1/2   1
//   theta_m:  -1  -5/6 -2/3 -1/2 -1/3 -1/6   0   1/6  1/3  1/2  2/3  5/6   1
//
//              |---------|---------|---------|---------|---------|---------|
//   xi:       0.0       1.0       2.0       3.0       4.0       5.0       0.0
//
static inline void SensorProfileUpdate(float theta_m, float omega_m,
                                       ZeroCrossingType crossing,
                                       const MotorParams *motor_params,
                                       SensorProfileParams *params,
                                       SensorProfileData *data,
                                       MotorAngleMeas *meas) {
  float npps = motor_params->num_pole_pairs_sens;
  float sin_f = meas->sin_m;
  float cos_f = meas->cos_m;

  // Generate an estimate for the mechanical angle at this time step. Note that
  // theta_m and omega_m are expected to come from the previous time step.
  theta_m += omega_m * g_period;

  // Map theta_m from roughly [-pi, pi] to roughly [0, 2*npps]. After wrapping,
  // the integer portion of xi will determine the interpolation index.
  float xi = (theta_m * (1.0f / PI_F) + 1.0f) * npps;
  SensorProfileInterp(xi, npps, &data->sin, &meas->sin_m);
  SensorProfileInterp(xi - 0.5f, npps, &data->cos, &meas->cos_m);

  // Update integrals.
  data->sin.sum += 0.5f * (sin_f + data->sin.val);
  data->sin.n += 1.0f;

  data->cos.sum += 0.5f * (cos_f + data->cos.val);
  data->cos.n += 1.0f;

  if (crossing != kZeroCrossingNone) {
    // Select the channel with the crossing.
    float val;
    int32_t i;
    SensorProfile *sensor = NULL;
    if (crossing == kZeroCrossingSinRise || crossing == kZeroCrossingSinFall) {
      val = sin_f;
      sensor = &data->sin;
      i = (int32_t)Wrapf(xi + 0.5f, 2.0f * npps);
    } else {  // crossing_type == kZeroCrossingCosFall or kZeroCrossingCosRise.
      val = cos_f;
      sensor = &data->cos;
      i = (int32_t)Wrapf(xi, 2.0f * npps);  // Find the closest index.
    }

    // Guard against division by zero, which should only happen if both sin and
    // cos inputs are held at zero.
    float delta_val = val - sensor->val;
    float eta = fabsf(delta_val) > FLT_EPSILON ? val / delta_val : 0.0f;

    // Remove the contribution from after the zero crossing.
    val = 0.5f * val * eta;
    sensor->sum -= val;
    sensor->n -= eta;

    // Update the enable latch. The profile algorithm is enabled when the number
    // of samples between zero-crossing events is below n_enable. It remains
    // enabled as long as the number of samples is below n_disable. These
    // sample count thresholds are set by omega_enable_threshold and
    // omega_disable_threshold.
    bool enable = sensor->enable;
    enable = enable || sensor->n < data->n_enable;
    enable = enable && sensor->n < data->n_disable;

    // Update the crossing index based off of the current mechanical angle.
    int32_t spin_dir = ISignf(omega_m);
    if (enable && sensor->enable && spin_dir == sensor->spin_dir_z) {
      // The index is lagged so averages on either side of i_z are used.
      int32_t i_z = sensor->i_z;
      float z_pole = params->filter_pole;

      // The offset is calulated by taking the mean value of the two most recent
      // half-cycles. This calculation only executes if the spin direction is
      // the same for both the current and previous zero-crossings; therefore,
      // mean and mean_z have opposite signs.
      // The measurement count sensor->n is always greater than zero.
      sensor->offset[i_z] = z_pole * sensor->offset[i_z] +
                            (1.0f - z_pole) * (sensor->sum + sensor->sum_z) /
                                (sensor->n + sensor->n_z);

      // The mean value of a sine function over a half-cycle is given by:
      // mean = (2 / pi * amplitude).
      // Since we want to find the scale using the average of the two most
      // recent half-cycles, the calculation becomes:
      // scale = |amplitude| = (pi / 2) * (|mean| + |mean_z| / 2).
      // For the same reason as above, mean and mean_z have opposite signs.
      sensor->scale[i_z] =
          z_pole * sensor->scale[i_z] +
          (1.0f - z_pole) * (PI_F / 4.0f) *
              fabsf(sensor->sum / sensor->n - sensor->sum_z / sensor->n_z);
    }

    sensor->sum_z = sensor->sum;
    sensor->n_z = sensor->n;
    sensor->sum = val;  // Reset the integration with the contributions from
    sensor->n = eta;    // after the zero crossing.

    sensor->i_zz = sensor->i_z;
    sensor->i_z = i;
    sensor->spin_dir_z = spin_dir;
    sensor->enable = enable;
  }

  data->sin.val = sin_f;
  data->cos.val = cos_f;
}

void MotorAngleSensorDiag(SensorProfileDiag *diag) {
  SensorProfileData *sensor = &g_state.sensor;

  int32_t i = sensor->sin.i_zz;
  diag->i_sin = i;
  diag->sin_offset = sensor->sin.offset[i];
  diag->sin_scale = sensor->sin.scale[i];

  i = sensor->cos.i_zz;
  diag->i_cos = i;
  diag->cos_offset = sensor->cos.offset[i];
  diag->cos_scale = sensor->cos.scale[i];
}

// Update derived parameters.
static void ZeroCrossingReset(float period, ZeroCrossingParams *params,
                              ZeroCrossingState *state) {
  // Cache the last filter pole and time step; only update if one changes.
  static float omega_filter_pole_last = 0.0f;
  static float period_last = 0.0f;
  if (params->omega_filter_pole != omega_filter_pole_last ||
      period != period_last) {
    float omega_filtered = FirstOrderFilterGet(&state->omega_filter);
    float omega_filter_zero = -INFINITY;
    FirstOrderFilterInit(1.0f, omega_filter_zero, params->omega_filter_pole,
                         period, omega_filtered, &params->omega_filter,
                         &state->omega_filter);

    omega_filter_pole_last = params->omega_filter_pole;
    period_last = period;
  }
}

// Initialize the zero crossing state based off of the present electrical angle.
// The next zero crossing is chosen assuming that the motor will begin spinning
// in the positive direction. A negative speed will result in an extra half
// electrical period delay when initially starting.
static void ZeroCrossingInit(float period, const MotorAngleMeas *meas,
                             ZeroCrossingParams *params,
                             ZeroCrossingState *state) {
  state->sin_last = meas->sin_m;
  state->cos_last = meas->cos_m;

  state->t32_last = Clock32GetCycles();
  uint32_t t32_last_crossing = state->t32_last - ZERO_CROSSING_DT_MAX;
  state->t32_cos_fall = t32_last_crossing;
  state->t32_cos_rise = t32_last_crossing;
  state->t32_sin_fall = t32_last_crossing;
  state->t32_sin_rise = t32_last_crossing;
  state->dt32 = ZERO_CROSSING_DT_MAX;

  // Assume the motor will start spinning in the positive direction.
  int32_t sin_m = meas->sin_m;  // Shorten name to prevent line wrapping.
  int32_t cos_m = meas->cos_m;
  state->next_crossing =
      sin_m >= 0 ? (cos_m >= 0 ? kZeroCrossingCosFall    // Quadrant 1, +x, +y.
                               : kZeroCrossingSinFall)   // Quadrant 2, -x.
                 : (cos_m <= 0 ? kZeroCrossingCosRise    // Quadrant 3, -y.
                               : kZeroCrossingSinRise);  // Quadrant 4.

  ZeroCrossingReset(period, params, state);
}

// Determine whether a crossing has occurred and return the appropriate value of
// kZeroCrossing{Sin/Cos}{Rise/Fall} or kZeroCrossingNone. The state machine
// additionally prevents noise from triggering a zero crossing.
static inline ZeroCrossingType ZeroCrossingSelectMode(
    int32_t sin_m, int32_t cos_m, ZeroCrossingState *state) {
  ZeroCrossingType crossing = kZeroCrossingNone;

  switch (state->next_crossing) {
    case kZeroCrossingCosFall:
      if (cos_m <= 0) {
        crossing = kZeroCrossingCosFall;
        if (sin_m > 0) {
          state->direction = 1;
          state->next_crossing = kZeroCrossingSinFall;
        } else {
          state->direction = -1;
          state->next_crossing = kZeroCrossingSinRise;
        }
      }
      break;
    case kZeroCrossingCosRise:
      if (cos_m >= 0) {
        crossing = kZeroCrossingCosRise;
        if (sin_m > 0) {
          state->direction = -1;
          state->next_crossing = kZeroCrossingSinFall;
        } else {
          state->direction = 1;
          state->next_crossing = kZeroCrossingSinRise;
        }
      }
      break;
    case kZeroCrossingSinFall:
      if (sin_m <= 0) {
        crossing = kZeroCrossingSinFall;
        if (cos_m > 0) {
          state->direction = -1;
          state->next_crossing = kZeroCrossingCosFall;
        } else {
          state->direction = 1;
          state->next_crossing = kZeroCrossingCosRise;
        }
      }
      break;
    case kZeroCrossingSinRise:
      if (sin_m >= 0) {
        crossing = kZeroCrossingSinRise;
        if (cos_m > 0) {
          state->direction = 1;
          state->next_crossing = kZeroCrossingCosFall;
        } else {
          state->direction = -1;
          state->next_crossing = kZeroCrossingCosRise;
        }
      }
      break;
    case kZeroCrossingNone:  // Fall-through intentional.
    default:
      assert(false);
  }
  return crossing;
}

// Helper function for ZeroCrossingUpdate that returns the number of cycles
// between t32 and t32_cross. If needed, t32_cross is shifted to keep the
// difference from exceeding ZERO_CROSSING_DT_MAX.
static inline uint32_t ZeroCrossingDeltaTime(uint32_t t32,
                                             uint32_t *t32_cross) {
  uint32_t dt32 = t32 - *t32_cross;
  if (dt32 > ZERO_CROSSING_DT_MAX) {
    dt32 = ZERO_CROSSING_DT_MAX;
    *t32_cross = t32 - ZERO_CROSSING_DT_MAX;
  }
  return dt32;
}

// Look for crossings in the sin or cos measurements, update the zero crossing
// state machine, and return the most recent zero crossing angular rate
// measurement.
static float ZeroCrossingUpdate(const MotorAngleMeas *meas,
                                const ZeroCrossingParams *params,
                                ZeroCrossingState *state,
                                ZeroCrossingType *crossing_type) {
  *crossing_type = ZeroCrossingSelectMode(meas->sin_m, meas->cos_m, state);

  uint32_t t32 = Clock32GetCycles();

  if (*crossing_type == kZeroCrossingNone) {
    // During long intervals without zero crossings, e.g. when the motors aren't
    // running, shift the last crossing point to prevent t32 - state->t32_*
    // from wrapping through 0.
    uint32_t dt32_cos_fall = ZeroCrossingDeltaTime(t32, &state->t32_cos_fall);
    uint32_t dt32_cos_rise = ZeroCrossingDeltaTime(t32, &state->t32_cos_rise);
    uint32_t dt32_sin_fall = ZeroCrossingDeltaTime(t32, &state->t32_sin_fall);
    uint32_t dt32_sin_rise = ZeroCrossingDeltaTime(t32, &state->t32_sin_rise);

    // This switch statement updates the state->dt32 with the greater of either
    // the last zero crossing time or next expected zero crossing (assuming it
    // happens now). This creates a smooth omega curve when slowing down and
    // allows the zero crossing scheme to approach zero when stopped, but is not
    // necessary when used with a derivative controlled transition.
    switch (state->next_crossing) {
      case kZeroCrossingCosFall:
        state->dt32 = state->dt32 > dt32_cos_fall ? state->dt32 : dt32_cos_fall;
        break;
      case kZeroCrossingCosRise:
        state->dt32 = state->dt32 > dt32_cos_rise ? state->dt32 : dt32_cos_rise;
        break;
      case kZeroCrossingSinFall:
        state->dt32 = state->dt32 > dt32_sin_fall ? state->dt32 : dt32_sin_fall;
        break;
      case kZeroCrossingSinRise:
        state->dt32 = state->dt32 > dt32_sin_rise ? state->dt32 : dt32_sin_rise;
        break;
      case kZeroCrossingNone:  // Fall-through intentional.
      default:
        assert(false);
        break;
    }
  } else {
    // Interpolate to find a more accurate estimate of the zero crossing.
    // TODO: Examine whether the interpolation can be removed.
    uint32_t t32_crossing = t32;
    float eta = 0.0f;
    if (*crossing_type == kZeroCrossingCosFall ||
        *crossing_type == kZeroCrossingCosRise) {
      // Explicitly check for divide by zero. The mode select should guarantee
      // that divide by zero can't happen (unless both inputs are held at zero),
      // but it's not that expensive to explicitly check for this condition.
      int32_t delta_cos = meas->cos_m - state->cos_last;
      eta = delta_cos != 0 ? (float)meas->cos_m / (float)delta_cos : 0.0f;
      t32_crossing -= (int32_t)((float)(t32 - state->t32_last) * eta);
    } else {  // crossing_type == kZeroCrossingSinFall or kZeroCrossingSinRise.
      int32_t delta_sin = meas->sin_m - state->sin_last;
      int32_t dt_sin = (int32_t)(t32 - state->t32_last) * meas->sin_m;
      t32_crossing -= delta_sin ? (uint32_t)(dt_sin / delta_sin) : 0U;
    }

    switch (*crossing_type) {
      case kZeroCrossingCosFall:
        state->dt32 = t32_crossing - state->t32_cos_fall;
        state->t32_cos_fall = t32_crossing;
        break;
      case kZeroCrossingCosRise:
        state->dt32 = t32_crossing - state->t32_cos_rise;
        state->t32_cos_rise = t32_crossing;
        break;
      case kZeroCrossingSinFall:
        state->dt32 = t32_crossing - state->t32_sin_fall;
        state->t32_sin_fall = t32_crossing;
        break;
      case kZeroCrossingSinRise:
        state->dt32 = t32_crossing - state->t32_sin_rise;
        state->t32_sin_rise = t32_crossing;
        break;
      case kZeroCrossingNone:  // Fall-through intentional.
      default:
        assert(false);
    }
  }

  // Give the compiler a hint that it should kick off the divide early.
  float omega_s = state->direction * k2PiClockFreq / (float)state->dt32;

  state->cos_last = meas->cos_m;
  state->sin_last = meas->sin_m;
  state->t32_last = t32;
  return FirstOrderFilter(omega_s, &params->omega_filter, &state->omega_filter);
}

// Update derived parameters.
static void DerivReset(float period, DerivParams *params, DerivState *state) {
  // Cache the last filter pole and time step; only update if one changes.
  static float omega_filter_pole_last = 0.0f;
  static float period_last = 0.0f;
  if (params->omega_filter_pole != omega_filter_pole_last ||
      period != period_last) {
    float omega_filtered = FirstOrderFilterGet(&state->omega_filter);
    float omega_filter_zero = -INFINITY;

    // A DC gain of 1/dt is used so the delta theta_m input acts as omega_m.
    FirstOrderFilterInit(1.0f / period, omega_filter_zero,
                         params->omega_filter_pole, period, omega_filtered,
                         &params->omega_filter, &state->omega_filter);

    omega_filter_pole_last = params->omega_filter_pole;
    period_last = period;
  }
}

static void DerivInit(float period, const MotorAngleMeas *meas,
                      const MotorParams *motor_params,
                      DerivParams *deriv_params, DerivState *state) {
  float theta_s = Atan2Lookup(meas->sin_m, meas->cos_m);
  state->theta_m = theta_s / motor_params->num_pole_pairs_sens;
  state->theta_m_error_int = 0.0f;

  DerivReset(period, deriv_params, state);
}

// Proportional-integral tracking filter on measured angle.
static float DerivUpdate(const MotorAngleMeas *meas,
                         const MotorParams *motor_params,
                         const DerivParams *params, DerivState *state) {
  float npps = motor_params->num_pole_pairs_sens;

  float theta_meas = Atan2Lookup(meas->sin_m, meas->cos_m);
  float theta_m_error = ModAngle(theta_meas - state->theta_m * npps) / npps;
  float theta_m_incr = params->theta_kp * theta_m_error +
                       params->theta_ki * state->theta_m_error_int;

  state->theta_m = ModAngle(state->theta_m + theta_m_incr);
  state->theta_m_error_int += g_period * theta_m_error;

  // The filter has a DC gain of 1/dt so that theta_m_incr acts as omega_m.
  return FirstOrderFilter(theta_m_incr, &params->omega_filter,
                          &state->omega_filter);
}

void MotorAngleReset(float period, const MotorParams *motor_params) {
  // Store in an intermediate variable for readability.
  MotorAngleParams *params = &g_motor_angle_params;

  DerivReset(period, &params->deriv, &g_state.deriv);
  SensorProfileReset(period, motor_params, &params->sensor, &g_state.sensor);
  ZeroCrossingReset(period, &params->zc, &g_state.zc);

  g_period = period;

  // Partially protect against invalid modifications.
  params->theta_offset_m =
      ModAngle(params->theta_offset_e / motor_params->num_pole_pairs_elec);
  if (params->omega_transition_zc <= params->omega_transition_deriv) {
    params->omega_transition_zc = params->omega_transition_deriv + 1.0f;
  }
}

// Initializes the angle sensors and angle estimation state.
void MotorAngleInit(float period, const MotorAngleMeas *meas,
                    const MotorParams *motor_params,
                    const MotorCalibParams *calib_params) {
  // Store in an intermediate variable for readability.
  MotorAngleParams *params = &g_motor_angle_params;

  DerivInit(period, meas, motor_params, &params->deriv, &g_state.deriv);
  SensorProfileInit(period, meas, motor_params, &params->sensor,
                    &g_state.sensor);
  ZeroCrossingInit(period, meas, &params->zc, &g_state.zc);

  params->omega_transition_deriv = 200.0f / motor_params->num_pole_pairs_sens;
  params->omega_transition_zc = 300.0f / motor_params->num_pole_pairs_sens;
  params->theta_offset_e = calib_params->angle_offset;
  params->theta_offset_m =
      calib_params->angle_offset / motor_params->num_pole_pairs_elec;
  g_period = period;
}

void MotorAngleUpdate(const MotorAngleMeas *angle_meas,
                      const MotorParams *motor_params, float *theta_mech,
                      float *omega_mech) {
  static float omega_m = 0.0f;  // Best estimate of mechanical speed.
  float npps = motor_params->num_pole_pairs_sens;

  // Store in an intermediate variable for readability.
  MotorAngleParams *params = &g_motor_angle_params;

  // Make a copy for modification.
  MotorAngleMeas meas = *angle_meas;

  if (meas.updated) {
    ZeroCrossingType crossing;
    float omega_zc_m =
        ZeroCrossingUpdate(&meas, &params->zc, &g_state.zc, &crossing) / npps;
    SensorProfileUpdate(g_state.deriv.theta_m, omega_m, crossing, motor_params,
                        &g_motor_angle_params.sensor, &g_state.sensor, &meas);
    float omega_deriv_m =
        DerivUpdate(&meas, motor_params, &params->deriv, &g_state.deriv);

    omega_m =
        Crossfadef(omega_deriv_m, omega_zc_m, fabsf(omega_deriv_m),
                   params->omega_transition_deriv, params->omega_transition_zc);
  } else {
    g_state.deriv.theta_m =
        ModAngle(g_state.deriv.theta_m + g_period * omega_m);
  }

  *omega_mech = omega_m;
  *theta_mech = ModAngle(g_state.deriv.theta_m - params->theta_offset_m);
}
