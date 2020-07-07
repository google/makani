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

#include "control/control_telemetry.h"

#include "avionics/common/tether_message_types.h"
#include "control/common.h"

ControlDebugMessage *GetControlDebugMessage() {
  static ControlDebugMessage message;
  return &message;
}

SmallControlTelemetryMessage *GetSmallControlTelemetry() {
  static SmallControlTelemetryMessage small_control_telemetry;
  return &small_control_telemetry;
}

ControlSlowTelemetry *GetControlSlowTelemetry() {
  static ControlSlowTelemetry control_slow_telemetry;
  return &control_slow_telemetry;
}

ControlTelemetry *GetControlTelemetry() {
  static ControlTelemetry control_telemetry;
  return &control_telemetry;
}

EstimatorTelemetry *GetEstimatorTelemetry() {
  return &GetControlTelemetry()->estimator;
}

HoverTelemetry *GetHoverTelemetry() { return &GetControlTelemetry()->hover; }

ManualTelemetry *GetManualTelemetry() { return &GetControlTelemetry()->manual; }

TransInTelemetry *GetTransInTelemetry() {
  return &GetControlTelemetry()->trans_in;
}

CrosswindTelemetry *GetCrosswindTelemetry() {
  return &GetControlTelemetry()->crosswind;
}

SyncTelemetry *GetSyncTelemetry() { return &GetControlTelemetry()->sync; }

Q7SlowStatusMessage *GetQ7SlowStatusMessage() {
  static Q7SlowStatusMessage status_message;
  return &status_message;
}

void ControlTelemetryToSmallControlTelemetryMessage(
    const ControlTelemetry *in, SmallControlTelemetryMessage *out) {
  ++out->sequence;  // Overwritten with AIO sequence number by core switch.
  out->controller_label = (uint8_t)in->controller_label;

  out->flags = 0x0;
  if (in->manual.auto_glide_active) {
    out->flags |= kTetherControlTelemetryFlagAutoGlideActive;
  }
  if (in->manual.release_latched) {
    out->flags |= kTetherControlTelemetryFlagReleaseLatched;
  }

  // Iterate through subsystems in fault.
  if (out->sequence % TETHER_CONTROL_TELEMETRY_DECIMATION == 0) {
    out->subsystem_faults = 0x0;
    for (int32_t i = 0; i < kNumSubsystems && out->subsystem_faults == 0x0;
         ++i) {
      out->subsystem = (uint8_t)((out->subsystem + 1U) % kNumSubsystems);
      out->subsystem_faults = (uint8_t)in->faults[out->subsystem].code;
    }
  }

  out->flight_mode = (uint8_t)in->flight_mode;
  int32_t next_flight_mode = GetNextFlightMode(in->flight_mode);
  out->flight_mode_gates = (uint16_t)in->flight_mode_gates[next_flight_mode];

  // Timers.
  out->flight_mode_time = (uint32_t)(in->flight_mode_time * 10.0);
  out->loop_time = (float)(in->finish_usec - in->start_usec) * 1e-6f;

  out->experiment_type = (uint8_t)in->state_est.experiment.active_type;
  out->experiment_case_id = in->state_est.experiment.case_id;

  // Pilot data.
  out->loop_count = 0;  // TODO: Compute loop count.
  out->loop_angle = (float)in->crosswind.loop_angle;
  out->airspeed = (float)in->state_est.apparent_wind.sph_f.v;
  out->alpha = (float)in->state_est.apparent_wind.sph_f.alpha;
  out->beta = (float)in->state_est.apparent_wind.sph_f.beta;
  {
    double roll, pitch, yaw;
    DcmToAngle(&in->state_est.dcm_g2b, kRotationOrderZyx, &yaw, &pitch, &roll);
    out->roll = (float)roll;
    out->pitch = (float)pitch;
    out->yaw = (float)yaw;
  }
  out->pqr[0] = (float)in->state_est.pqr_f.x;
  out->pqr[1] = (float)in->state_est.pqr_f.y;
  out->pqr[2] = (float)in->state_est.pqr_f.z;
  out->pos_g[0] = (float)in->state_est.Xg.x;
  out->pos_g[1] = (float)in->state_est.Xg.y;
  out->pos_g[2] = (float)in->state_est.Xg.z;
  out->vel_g[0] = (float)in->state_est.Vg.x;
  out->vel_g[1] = (float)in->state_est.Vg.y;
  out->vel_g[2] = (float)in->state_est.Vg.z;

  // Crosswind status.
  out->target_pos_cw[0] = (float)in->crosswind.target_pos_cw.x;
  out->target_pos_cw[1] = (float)in->crosswind.target_pos_cw.y;
  out->current_pos_cw[0] = (float)in->crosswind.current_pos_cw.x;
  out->current_pos_cw[1] = (float)in->crosswind.current_pos_cw.y;
  out->delta_aileron = (float)in->deltas.aileron;
  out->delta_elevator = (float)in->deltas.elevator;
  out->delta_rudder = (float)in->deltas.rudder;
  out->delta_inboard_flap = (float)in->deltas.inboard_flap;

  // Hover dynamics.
  out->tension = (float)in->state_est.tether_force_b.sph.tension;
  out->tension_command = (float)in->hover.tension_cmd;
  out->thrust = (float)in->thrust_moment.thrust;
  out->thrust_avail = (float)in->thrust_moment_avail.thrust;
  out->moment[0] = (float)in->thrust_moment.moment.x;
  out->moment[1] = (float)in->thrust_moment.moment.y;
  out->moment[2] = (float)in->thrust_moment.moment.z;
  out->moment_avail[0] = (float)in->thrust_moment_avail.moment.x;
  out->moment_avail[1] = (float)in->thrust_moment_avail.moment.y;
  out->moment_avail[2] = (float)in->thrust_moment_avail.moment.z;
  out->gain_ramp_scale = (float)in->hover.gain_ramp_scale;
  out->force_detwist_turn_once = (uint8_t)in->state_est.force_detwist_turn_once;
}
