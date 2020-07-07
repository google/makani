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

#include "gs/monitor/charts/charts_controller.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/geometry.h"
#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator.h"
#include "control/estimator/estimator_types.h"
#include "control/hover/hover_tension.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_params.h"
#include "gs/monitor/monitor_types.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/chart.h"
#include "gs/monitor/widgets/indicator.h"
#include "gs/monitor/widgets/indicator_chart.h"

void UpdateAirspeed(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Airspeed [m/s]");
    chart_set_num_lines(ich->chart, 1);
    chart_set_yrange(ich->chart, 0.0, 60.0);
  }

  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  double airspeed = ct->state_est.apparent_wind.sph.v;
  ApparentWindSolutionType sol_type =
      (ApparentWindSolutionType)ct->state_est.apparent_wind.solution_type;

  const MonEstimatorParams *params = &GetMonitorParams()->est;
  if ((ct->flight_mode == kFlightModeOffTether &&
       airspeed < params->airspeed.low) ||
      ((ct->flight_mode == kFlightModeCrosswindNormal ||
        ct->flight_mode == kFlightModeCrosswindPrepTransOut) &&
       airspeed > params->airspeed.high)) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }

  MON_PRINTF(ich->indicator, "%5.1f %s", airspeed,
             ApparentWindSolutionTypeToString(sol_type));
  chart_add_points(ich->chart, fd->t, &airspeed);
}

void UpdateAltitude(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Altitude [m]");
    chart_set_num_lines(ich->chart, 3);
    chart_set_yrange(ich->chart, -15.0, 50.0);
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  double alt = -ct->state_est.Xg.z;
  double alt_est_baro_diff = alt + ct->estimator.baro.Xg_z;
  double alt_kalman_cov = ct->estimator.pos_baro_state.cov_Xg_z_bias;
  // Pre-flight: simple altitude check
  // During flight: sensor conflict or filter covariance related warnings
  const MonContParams *params = &GetMonitorParams()->control;
  const Vec3 *perched_wing_pos_p =
      &GetControlParams()->hover.path.perched_wing_pos_p;
  double alt_perched = -perched_wing_pos_p->z;
  if (!CheckMotorsOn() && alt_perched - 0.1 < alt && alt < alt_perched + 0.1) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else if ((ct->flight_mode <= kFlightModeHoverFullLength ||
              (ct->flight_mode >= kFlightModeHoverTransOut &&
               ct->flight_mode <= kFlightModeHoverDescend)) &&
             (fabs(alt_est_baro_diff) > params->alt_sens_diff.high ||
              alt_kalman_cov > params->alt_kalman_cov.high)) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }
  MON_PRINTF(ich->indicator, "Alt:  % 0.3f\nDiff: % 0.1f\nCov:  % 0.1f", alt,
             alt_est_baro_diff, alt_kalman_cov);
  double altitudes[3] = {alt, alt_est_baro_diff, alt_kalman_cov};
  chart_add_points(ich->chart, fd->t, altitudes);
}

void UpdateCrosswindDeltas(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Deltas [deg]");
    chart_set_num_lines(ich->chart, 4);
    chart_set_yrange(ich->chart, -30.0, 30.0);
  }

  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);

  double deltas_deg[] = {
      ct->deltas.aileron * 180.0 / PI, ct->deltas.inboard_flap * 180.0 / PI,
      ct->deltas.elevator * 180.0 / PI, ct->deltas.rudder * 180.0 / PI};

  MON_PRINTF(ich->indicator,
             "Ail:  % 0.2f\n"
             "Flap: % 0.2f\n"
             "Ele:  % 0.2f\n"
             "Rud:  % 0.2f\n",
             deltas_deg[0], deltas_deg[1], deltas_deg[2], deltas_deg[3]);
  chart_add_points(ich->chart, fd->t, deltas_deg);
}

void UpdateEngageAltitude(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "alt [m]");
    chart_set_num_lines(ich->chart, 2);
    chart_set_yrange(ich->chart, 0, 90);
  }

  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);

  double alt[2] = {-ct->state_est.Xg.z, -ct->hover.wing_pos_g_cmd.z};

  MON_PRINTF(ich->indicator, "alt:  %0.2f\nalt_req:  %0.2f", alt[0], alt[1]);
  chart_add_points(ich->chart, fd->t, alt);
}

void UpdateEstimatorAttitudeDiff(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Att. Diff. [rad]");
    chart_set_num_lines(ich->chart, kNumWingImus);
    chart_set_yrange(ich->chart, 0.0, 0.3);
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  Quat q;
  DcmToQuat(&ct->state_est.dcm_g2b, &q);
  double attitude_diff[kNumWingImus];
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    double dot = QuatDot(&q, &ct->estimator.q_g2b[i]);
    // See the comment for the QuatDot function in quaternion.h to see
    // why this calculates the rotation angle between two quaternions.
    attitude_diff[i] = Acos(1 - 2 * (1 - dot * dot));
  }
  MON_PRINTF(ich->indicator, "A: %0.3f\nB: %0.3f\nC: %0.3f",
             attitude_diff[kWingImuA], attitude_diff[kWingImuB],
             attitude_diff[kWingImuC]);

  const MonEstimatorParams *params = &GetMonitorParams()->est;
  if (MaxArray(attitude_diff, ARRAYSIZE(attitude_diff), NULL) >
      params->max_attitude_diff) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }
  chart_add_points(ich->chart, fd->t, attitude_diff);
}

void UpdateEstimatorTetherForce(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Tether Force");
    chart_set_num_lines(ich->chart, 3);
    chart_set_yrange(ich->chart, -30.0, 30.0);
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  double values[] = {
      ct->state_est.tether_force_b.sph.tension / 1e3,
      ct->state_est.tether_force_b.sph.roll * 180.0 / PI,
      ct->state_est.tether_force_b.sph.pitch * 180.0 / PI,
  };
  MON_PRINTF(ich->indicator,
             "T  [kN]: % 0.2f\n"
             "R [deg]: % 0.2f\n"
             "P [deg]: % 0.2f",
             values[0], values[1], values[2]);
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  chart_add_points(ich->chart, fd->t, values);
}

void UpdateHoverAngles(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Hover angles [rad]");
    chart_set_num_lines(ich->chart, 3);
    chart_set_yrange(ich->chart, -PI / 4.0, PI / 4.0);
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  double rpy[] = {ct->hover.angles.x, ct->hover.angles.y, ct->hover.angles.z};
  const MonEstimatorParams *params = &GetMonitorParams()->est;
  // Only issue warnings in "non-dynamic" hover modes.
  if ((ct->flight_mode <= kFlightModeHoverFullLength ||
       (ct->flight_mode >= kFlightModeHoverTransOut &&
        ct->flight_mode <= kFlightModeHoverDescend)) &&
      (rpy[0] < params->hover_angles[0].low ||
       rpy[0] > params->hover_angles[0].high ||
       rpy[1] < params->hover_angles[1].low ||
       rpy[1] > params->hover_angles[1].high ||
       rpy[2] < params->hover_angles[2].low ||
       rpy[2] > params->hover_angles[2].high)) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }
  MON_PRINTF(ich->indicator, "Roll:  % 0.2f\nPitch: % 0.2f\nYaw:   % 0.2f",
             rpy[0], rpy[1], rpy[2]);
  chart_add_points(ich->chart, fd->t, rpy);
}

void UpdateHoverPathCommand(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Path cmd [m]");
    chart_set_num_lines(ich->chart, 3);
    chart_set_yrange(ich->chart, -10.0, 10.0);
  }
  if (!CheckControllerTelemetry()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  const MonEstimatorParams *params = &GetMonitorParams()->est;

  double Xg_norm = Vec3Norm(&ct->state_est.Xg);
  double Xg_norm_z1 = Vec3Norm(&ct_z1->state_est.Xg);
  if (Xg_norm < params->close_range && !(Xg_norm_z1 < params->close_range)) {
    chart_set_yrange(ich->chart, -params->close_range, params->close_range);
  }
  if (Xg_norm >= params->close_range && Xg_norm < params->mid_range &&
      !(Xg_norm_z1 >= params->close_range && Xg_norm_z1 < params->mid_range)) {
    chart_set_yrange(ich->chart, -params->mid_range, params->mid_range);
  }
  if (Xg_norm >= params->mid_range && !(Xg_norm_z1 >= params->mid_range)) {
    chart_set_yrange(ich->chart, -params->full_range, params->full_range);
  }
  double wing_pos_g_cmd[] = {ct->hover.wing_pos_g_cmd.x,
                             ct->hover.wing_pos_g_cmd.y,
                             ct->hover.wing_pos_g_cmd.z};
  MON_PRINTF(ich->indicator,
             "xg_cmd: % 0.2f\n"
             "yg_cmd: % 0.2f\n"
             "zg_cmd: % 0.2f",
             wing_pos_g_cmd[0], wing_pos_g_cmd[1], wing_pos_g_cmd[2]);
  chart_add_points(ich->chart, fd->t, wing_pos_g_cmd);
}

void UpdateHoverPathErrors(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Path error [rad]");
    chart_set_num_lines(ich->chart, 2);
    chart_set_yrange(ich->chart, -0.3, 0.3);
  }
  if (!CheckControllerTelemetry()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  double angle_errors[] = {
      Wrap(atan2(ct->hover.wing_pos_g_cmd.y, ct->hover.wing_pos_g_cmd.x) -
               atan2(ct->state_est.Xg.y, ct->state_est.Xg.x),
           -PI, PI),
      Wrap(atan2(-ct->hover.wing_pos_g_cmd.z,
                 Vec3XyNorm(&ct->hover.wing_pos_g_cmd)) -
               atan2(-ct->state_est.Xg.z, Vec3XyNorm(&ct->state_est.Xg)),
           -PI, PI)};
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  MON_PRINTF(ich->indicator,
             "Azi err: % 0.2f\n"
             "Ele err: % 0.2f\n",
             angle_errors[0], angle_errors[1]);
  chart_add_points(ich->chart, fd->t, angle_errors);
}

void UpdateHoverPositionErrors(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Pos. error [m]");
    chart_set_num_lines(ich->chart, 3);
    chart_set_yrange(ich->chart, -10.0, 10.0);
  }
  if (!CheckControllerTelemetry()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  double position_errors[] = {
      -(ct->hover.wing_pos_g_cmd.z - ct->state_est.Xg.z),
      ct->hover.wing_pos_b_error.y, ct->hover.wing_pos_b_error.z};

  const HoverPositionParams *params = &GetControlParams()->hover.position;
  // We use the angle feedback being greater than the maximum angle
  // feedback from the position alone as an indicator that the
  // tangential feedback is saturated.
  //
  // TODO: Determine better method of detecting saturation.
  if (fabs(ct->hover.angles_fb.z) >= params->max_pos_angle_fb) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }
  MON_PRINTF(ich->indicator,
             "alt: % 0.2f\n"
             "tng: % 0.2f\n"
             "rad: % 0.2f",
             position_errors[0], position_errors[1], position_errors[2]);
  chart_add_points(ich->chart, fd->t, position_errors);
}

void UpdateHoverTension(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Tension [N]");
    chart_set_num_lines(ich->chart, 2);
    chart_set_yrange(ich->chart, -100.0, 7000.0);
  }
  if (!CheckControllerTelemetry()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  double tensions[] = {ct->hover.tension_cmd,
                       ct->state_est.tether_force_b.sph.tension};

  const HoverTensionParams *params = &GetControlParams()->hover.tension;
  PidParams tension_pid;
  CrossfadePidParams(&params->tension_hard_pid, &params->tension_soft_pid,
                     ct->state_est.winch.payout, params->hard_spring_payout,
                     params->soft_spring_payout, &tension_pid);

  // Define warning strings.
  const char no_warning[] = "";
  const char int_saturated_str[] = "int. sat.";
  const char pitch_saturated_str[] = "pitch sat.";

  // Default to green and no warning.
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  const char *warning_str = no_warning;

  // Check for saturated integrator or output pitch command.
  if (IsSaturated(ct->hover.int_pitch, tension_pid.int_output_min,
                  tension_pid.int_output_max)) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
    warning_str = int_saturated_str;
  } else if (IsSaturated(ct->hover.pitch_ff + ct->hover.pitch_fb,
                         ct->hover.pitch_min, ct->hover.pitch_max)) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
    warning_str = pitch_saturated_str;
  }

  MON_PRINTF(ich->indicator,
             "Cmd:  % 0.0f\n"
             "Meas: % 0.0f\n"
             "%s",
             tensions[0], tensions[1], warning_str);
  chart_add_points(ich->chart, fd->t, tensions);
}

void UpdateHoverVelocityErrors(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Vel. error [m/s]");
    chart_set_num_lines(ich->chart, 3);
    chart_set_yrange(ich->chart, -3.0, 3.0);
  }
  if (!CheckControllerTelemetry()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  double velocity_errors[] = {
      -(ct->hover.wing_vel_g_cmd.z - ct->state_est.Vg.z),
      ct->hover.wing_vel_b_error.y, ct->hover.wing_vel_b_error.z};

  const HoverPositionParams *params = &GetControlParams()->hover.position;
  // We use the angle feedback being greater than the maximum angle
  // feedback from the velocity alone as an indicator that the
  // tangential feedback is saturated.
  //
  // TODO: Determine better method of detecting saturation.
  if (fabs(ct->hover.angles_fb.z) >= params->max_vel_angle_fb) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }
  MON_PRINTF(ich->indicator,
             "v_alt: % 0.2f\n"
             "v_tng: % 0.2f\n"
             "v_rad: % 0.2f",
             velocity_errors[0], velocity_errors[1], velocity_errors[2]);
  chart_add_points(ich->chart, fd->t, velocity_errors);
}

void UpdateHoverWindDir(IndicatorChart *ich, int32_t init) {
  // Plot the wind and perch directions, which must be aligned for launch.
  if (init) {
    indicator_set_label(ich->indicator, "Wind dir. [deg]");
    chart_set_num_lines(ich->chart, 2);
    chart_set_yrange(ich->chart, -180.0, 180.0);
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  // Add 180 degrees to the perch azimuth here, as that is the
  // configuration during most of the hover modes.  (Once the tether
  // is payed out completely, the perch rotates through 180 degrees.)
  double wind_dir_deg = 180.0 / PI * ct->state_est.wind_g.dir_f;
  double perch_dir_deg =
      180.0 / PI * Wrap(ct->state_est.perch_azi.angle + PI, -PI, PI);

  const HoverModeParams *params = &GetControlParams()->hover.mode;

  double perch_wind_misalignment =
      fabs(Wrap(ct->state_est.perch_azi.angle + PI - ct->state_est.wind_g.dir_f,
                -PI, PI));

  // Display a warning if either sensor is invalid, or if the perch
  // and wind are misaligned in flight modes where we expect them to
  // be aligned.
  if (!ct->state_est.wind_g.valid || !ct->state_est.perch_azi.valid ||
      ((ct->flight_mode == kFlightModePilotHover ||
        ct->flight_mode == kFlightModePerched ||
        ct->flight_mode == kFlightModeHoverAscend ||
        ct->flight_mode == kFlightModeHoverPayOut ||
        ct->flight_mode == kFlightModeHoverReelIn ||
        ct->flight_mode == kFlightModeHoverDescend) &&
       (perch_wind_misalignment >
        params->max_perch_wind_misalignment_for_ascend))) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }

  MON_PRINTF(ich->indicator,
             "Wind dir:  % 4.0f\n"
             "Perch dir: % 4.0f",
             wind_dir_deg, perch_dir_deg);

  double directions[] = {wind_dir_deg, perch_dir_deg};
  chart_add_points(ich->chart, fd->t, directions);
}

void UpdatePitotAngles(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Pitot angles [deg]");
    chart_set_num_lines(ich->chart, 2);
    chart_set_yrange(ich->chart, -20.0, 20.0);
  }
  double alpha, beta;
  if (CheckControllerRunning()) {
    alpha = ct->state_est.apparent_wind.sph.alpha;
    beta = ct->state_est.apparent_wind.sph.beta;
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  const MonEstimatorParams *params = &GetMonitorParams()->est;
  if ((AnyCrosswindFlightMode(ct->flight_mode) ||
       ct->flight_mode == kFlightModeOffTether) &&
      (alpha < params->alpha.low || beta > params->beta.high)) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }
  MON_PRINTF(ich->indicator, "Alpha: % 5.1f\nBeta:  % 5.1f", 180.0 / PI * alpha,
             180.0 / PI * beta);
  double angles[2] = {180.0 / PI * alpha, 180.0 / PI * beta};
  chart_add_points(ich->chart, fd->t, angles);
}

void UpdateTension(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Tension [kN]");
    chart_set_num_lines(ich->chart, 1);
    chart_set_yrange(ich->chart, 0.0, 250.0);
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  if (ct->flight_mode != ct_z1->flight_mode) {
    if (AnyCrosswindFlightMode(ct->flight_mode)) {
      chart_set_yrange(ich->chart, 0.0, 250.0);
    } else if (AnyAutoHoverFlightMode(ct->flight_mode)) {
      chart_set_yrange(ich->chart, 0.0, 100.0);
    }
  }

  const MonTetherParams *params = &GetMonitorParams()->tether;
  double tension = ct->state_est.tether_force_b.sph.tension;
  double tension_kN = tension / 1000.0;
  char action_str[16] = "";
  if (tension > params->tension.very_high) {
    strncpy(action_str, "ABORT\n", sizeof(action_str));
    indicator_set_state(ich->indicator, INDICATOR_STATE_ERROR);
  } else if (tension > params->tension.high) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }
  MON_PRINTF(ich->indicator, "%s%5.1f", action_str, tension_kN);
  chart_add_points(ich->chart, fd->t, &tension_kN);
}

void UpdateWindDir(IndicatorChart *ich, int32_t init) {
  // plot both wind dir and tether dir, warning on diff
  if (init) {
    indicator_set_label(ich->indicator, "Rel. wind dir. [deg]");
    chart_set_num_lines(ich->chart, 2);
    chart_set_yrange(ich->chart, -50.0, 50.0);
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  double wind_dir = 180.0 / PI * atan2(-ct->state_est.wind_g.vector.y,
                                       -ct->state_est.wind_g.vector.x);
  double tether_azi =
      180.0 / PI * atan2(-ct->state_est.Xg.y, -ct->state_est.Xg.x);
  double rel_dir = wind_dir - tether_azi;
  const MonEnviroParams *params = &GetMonitorParams()->environment;
  if (fabs(rel_dir) > params->rel_direction.high) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }
  MON_PRINTF(ich->indicator, "Wind (g): % 0.0f\nDiff:     % 0.0f", wind_dir,
             rel_dir);
  double directions[2] = {wind_dir, rel_dir};
  chart_add_points(ich->chart, fd->t, directions);
}

void UpdateWingPos(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Wing pos [m]");
    chart_set_num_lines(ich->chart, 3);
    chart_set_yrange(ich->chart, -5.0, 15.0);
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  const MonEstimatorParams *params = &GetMonitorParams()->est;

  double nXg = Vec3Norm(&ct->state_est.Xg);
  double nXg_z1 = Vec3Norm(&ct_z1->state_est.Xg);
  if (nXg < params->close_range && !(nXg_z1 < params->close_range))
    chart_set_yrange(ich->chart, -params->close_range, params->close_range);
  if (nXg >= params->close_range && nXg < params->mid_range &&
      !(nXg_z1 >= params->close_range && nXg_z1 < params->mid_range))
    chart_set_yrange(ich->chart, -params->mid_range, params->mid_range);
  if (nXg >= params->mid_range && !(nXg_z1 >= params->mid_range))
    chart_set_yrange(ich->chart, -params->full_range, params->full_range);

  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  MON_PRINTF(ich->indicator,
             "xg:   % 8.2f\nyg:   % 8.2f\nzg:   % 8.2f\n|Xg|: % 8.2f",
             ct->state_est.Xg.x, ct->state_est.Xg.y, ct->state_est.Xg.z,
             Vec3Norm(&ct->state_est.Xg));
  double Xg[3] = {ct->state_est.Xg.x, ct->state_est.Xg.y, ct->state_est.Xg.z};
  chart_add_points(ich->chart, fd->t, Xg);
}
