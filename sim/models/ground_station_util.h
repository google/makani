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

#ifndef SIM_MODELS_GROUND_STATION_UTIL_H_
#define SIM_MODELS_GROUND_STATION_UTIL_H_

#include "common/c_math/vec3.h"
#include "control/system_types.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_types.h"

bool ContactPerchPanel(const Vec3 &contactor_pos_p,
                       const ReferenceFrame &perch_frame,
                       const ReferenceFrame &panel_frame,
                       const PanelSimParams &panel_sim_params,
                       Vec3 *collision_pos_p);

// Determines the length of tether wrapped around the racetrack.
//
// The racetrack is modeled as a helix with varying radius. We parameterize it
// in terms of
//     tau = drum_angles.racetrack_high - drum_angle,
// so the tether begins wrapping on the racetrack at tau=0 and wraps onto it as
// tau increases.
//
// The helix is then parameterized as
//     p(tau) = [dx_dtau * tau, R(tau) * sin(tau), R(tau) * cos(tau)],
//     where R(tau) = R_0 + dr_dtau * tau.
// (The y- and z-coordinates here don't necessarily align with the drum frame,
// but that is unimportant.) R_0 is the starting radius of the racetrack.
//
// The arc length integrand is then
//     |p'(tau)| = sqrt(a*tau**2 + b*tau + c),
// where
//     a = dr_dtau**2,
//     b = 2 * R_0 * dr_dtau,
//     c = R_0**2 * dx_dtau**2 * dr_dtau**2.
class RacetrackTetherIntegrator {
  friend class RacetrackTetherIntegratorTest;

 public:
  RacetrackTetherIntegrator(const Gs02Params &params,
                            const Gs02SimParams &sim_params);

  // Returns the length of tether wrapped on the racetrack for a given drum
  // angle.
  double WrappedLength(double drum_angle) const;

 private:
  // Returns a value of a particular antiderivative of the arc length integrand.
  double Antiderivative(double tau) const;

  // Coefficients of the arc length integrand, as in the class description.
  double a_;
  double b_;
  double c_;

  // Maximum drum angle at which the tether is anchored to the racetrack.
  double theta_high_;

  // Value of tau (see class description for definition) at which the tether
  // has been fully wrapped onto the racetrack.
  double tau_max_;
};

#endif  // SIM_MODELS_GROUND_STATION_UTIL_H_
