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

#include "control/manual/manual_output.h"

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/linalg.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/system_params.h"

void ManualOutputStep(const Deltas *deltas, const ThrustMoment *thrust_moment,
                      const ManualOutputParams *params,
                      const double *flight_mode_time,
                      ControlOutput *control_output) {
  ThrustMoment thrust_moment_avail;
  double v_app_locals[kNumMotors];
  double rotors[kNumMotors];
  // TODO: See whether we really want to call MixRotors when off-tether.
  MixRotors(thrust_moment, &params->thrust_moment_weights, 0.0, &kVec3Zero,
            kStackingStateNormal, true, g_sys.phys->rho, g_sys.rotors,
            g_cont.rotor_control, rotors, &thrust_moment_avail, v_app_locals);

  Deltas deltas_available;
  MixFlaps(deltas, params->flap_offsets, params->lower_flap_limits,
           params->upper_flap_limits, control_output->flaps, &deltas_available);

  // Winch velocity is always zero in manual.
  control_output->winch_vel_cmd = 0.0;

  // The detwist servo is unused in manual mode.
  control_output->detwist_cmd = 0.0;

  // The tether must be released in manual mode.
  control_output->tether_release = true;

  // Motors stay enabled off tether to bring voltage down.
  control_output->stop_motors = false;

  // These values are set by the outer control system.
  control_output->run_motors = false;
  control_output->sync.sequence = 0U;
  control_output->sync.flight_mode = -1;

  // Bound allowable speeds.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    if (*flight_mode_time >= params->kes_delay) {
      // After release, increase torque to bring down voltage.
      control_output->motor_torque[i] = params->kes_torque;
    } else {
      // Command torque to zero to minimize arcing during release.
      control_output->motor_torque[i] = 0.0;
    }

    // Open up speed commands fully.  We allow zero rad/sec for the lower speed
    // command but this is overridden in the LimitOutput function in
    // control_output.c.
    // TODO: Modify LimitOutput to be more consistent with stalling
    // propellers in off-tether flight.
    control_output->motor_speed_upper_limit[i] =
        g_cont.rotor_control->max_speeds[i];
    control_output->motor_speed_lower_limit[i] = 0.0;  // Stalling okay
  }

  // Keep the ground station in high-tension mode, telling it to hold our last
  // crosswind target position, so the azimuth brake will stay engaged.
  //
  // TODO(b/70903014): Consider edge cases - could we ever enter this mode when
  // the GS is not already in HighTension? What if the azimuth isn't aligned
  // when we enter manual mode, so the brake is not engaged?
  control_output->gs_mode_request = kGroundStationModeHighTension;

  control_output->gs_azi_cmd.target = 0.0;
  control_output->gs_azi_cmd.dead_zone = 0.0;
  control_output->gs_unpause_transform = false;
  control_output->hold_gs_azi_cmd = true;

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->deltas = *deltas;
  ct->deltas_avail = deltas_available;
  ct->thrust_moment = *thrust_moment;
  ct->thrust_moment_avail = thrust_moment_avail;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    ct->v_app_locals[i] = v_app_locals[i];
  }
}
