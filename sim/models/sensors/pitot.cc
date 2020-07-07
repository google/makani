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

#include "sim/models/sensors/pitot.h"

#include <math.h>

#include <vector>

#include "avionics/common/avionics_messages.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/sensor_util.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/environment.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/models/signals/measurement.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"

Pitot::Pitot(const Environment &environment, const Wing &wing,
             PitotSensorLabel label, FlightComputerLabel fc_label,
             const PitotParams &pitot_params,
             const PitotSimParams &pitot_sim_params, FaultSchedule *faults)
    : Sensor("Pitot[" + std::to_string(static_cast<int32_t>(label)) + "]"),
      label_(label),
      dcm_b2actual_(pitot_params.dcm_b2p),
      pitot_params_(pitot_params),
      pitot_sim_params_(pitot_sim_params),
      fc_label_(fc_label),
      wing_(wing),
      environment_(environment),
      total_rotor_thrust_(new_derived_value(), "total_rotor_thrust"),
      actual_P_dyn_(new_discrete_state(), "actual_P_dyn", 0.0),
      actual_P_stat_(new_discrete_state(), "actual_P_stat", 0.0),
      actual_P_alpha_(new_discrete_state(), "actual_P_alpha", 0.0),
      actual_P_beta_(new_discrete_state(), "actual_P_beta", 0.0),
      P_dyn_(full_name(), "P_dyn", pitot_sim_params.ts,
             {pitot_sim_params.dyn_sensor}, actual_P_dyn_, faults),
      P_stat_(full_name(), "P_stat", pitot_sim_params.ts,
              {pitot_sim_params.stat_sensor}, actual_P_stat_, faults),
      P_alpha_(full_name(), "P_alpha", pitot_sim_params.ts,
               {pitot_sim_params.alpha_sensor}, actual_P_alpha_, faults),
      P_beta_(full_name(), "P_beta", pitot_sim_params.ts,
              {pitot_sim_params.beta_sensor}, actual_P_beta_, faults) {
  CHECK_LE(0, fc_label_);
  CHECK_GT(kNumFlightComputers, fc_label_);
  set_sub_models({&P_stat_, &P_dyn_, &P_alpha_, &P_beta_});

  // Compute body-to-actual DCM.
  if (*g_sim.sim_opt & kSimOptImperfectSensors) {
    AngleToDcm(pitot_sim_params_.yaw_offset, pitot_sim_params_.pitch_offset,
               0.0, kRotationOrderZyx, &dcm_b2actual_);
    Mat3Mat3Mult(&dcm_b2actual_, &pitot_params_.dcm_b2p, &dcm_b2actual_);
  }

  SetupDone();
}

namespace internal {

double SphereDynamicPressure(double air_density, const Vec3 &apparent_wind_p,
                             const Vec3 &port_direction_p) {
  DCHECK_GE(1e-6, fabs(1.0 - Vec3Norm(&port_direction_p)));
  Vec3 tmp;
  // The coefficient of pressure on a sphere is given by:
  //
  //  C_P = 1.0 - (9.0 / 4.0) * sin(theta)^2.
  //
  // We avoid trigonometric equations here by exploiting the fact that
  // the cross product between vectors u and v is ||u|| * ||v|| *
  // sin(theta), where theta is the angle between the vectors.  Since
  // we only care about sin(theta)^2, we are not worried about the
  // sense of the angle here.
  return 0.5 * air_density *
         (Vec3NormSquared(&apparent_wind_p) -
          (9.0 / 4.0) * Vec3NormSquared(Vec3Cross(&apparent_wind_p,
                                                  &port_direction_p, &tmp)));
}

// The Aeroprobe five-port Pitot tube tip is hemispherical and we
// model the pressure distribution on the tip by the distribution on a
// sphere using the analytical distribution for an inviscid,
// incompressible flow (see SphereDynamicPressure above).
//
//  Cross section of Pitot tube.            Frontal view of the probe.
//
//        ..(1)..                                    .---------.
//     (5)   |   (4)                                '    (5)    `
//    .` \ a | a / `.                              '             '
//   `    \  |  /    `                             | (3) (1) (2) |
//  `      \ | /      `    a: port_angle           |             |
//  '       \|/       '                            ',    (4)    ,'
//  |        x        |                              '---------'
//
// The "dynamic pressure" measurement reports the pressure difference
// between port 1 and the static-pressure port. The pressure
// difference between ports 5 and 4 is reported as the alpha pressure
// reading.  The difference between ports 3 and 2 is reported as the
// beta pressure reading.
void CalcFivePortPressures(double air_density, double port_angle,
                           const Vec3 &apparent_wind_p, double *p_dynamic,
                           double *p_alpha, double *p_beta) {
  *p_dynamic = SphereDynamicPressure(air_density, apparent_wind_p, kVec3X);
  *p_alpha = SphereDynamicPressure(air_density, apparent_wind_p,
                                   {cos(port_angle), 0.0, sin(port_angle)}) -
             SphereDynamicPressure(air_density, apparent_wind_p,
                                   {cos(port_angle), 0.0, -sin(port_angle)});
  *p_beta = SphereDynamicPressure(air_density, apparent_wind_p,
                                  {cos(port_angle), sin(port_angle), 0.0}) -
            SphereDynamicPressure(air_density, apparent_wind_p,
                                  {cos(port_angle), -sin(port_angle), 0.0});
}

void CalcInflowFromRotorsAtPitot(double air_density, double thrust,
                                 double freestream_vel,
                                 const PitotSimParams &params, Vec3 *inflow_b) {
  // Calculate the induced velocity at the rotor disks using momentum
  // theory (see Rotorcraft Aeromechanics Eq. 4.4 and Eq. 4.8).  Note
  // that momentum theory breaks down in the vortex ring state and
  // turbulent wake state.
  double hover_induced_vel =
      sqrt(fabs(thrust) / (2.0 * air_density * params.total_rotor_area));

  // Do not use the Sign function here because the induced velocity
  // calculation relies on sign(0) = 1.
  double sign_freestream_vel = (freestream_vel >= 0.0) ? 1.0 : -1.0;

  double induced_vel =
      -freestream_vel / 2.0 +
      sign_freestream_vel *
          Sqrt((freestream_vel / 2.0) * (freestream_vel / 2.0) +
               sign_freestream_vel * Sign(thrust) * hover_induced_vel *
                   hover_induced_vel);

  // Estimate the induced velocity at the pitot using an empirically
  // determined ratio to the induced velocity at the rotor disk.
  double induced_vel_at_pitot =
      params.induced_vel_at_pitot_fraction * induced_vel;

  // The induced velocity is defined to be positive during hover
  // above.
  Vec3Scale(&params.rotor_axis, -induced_vel_at_pitot, inflow_b);
}

}  // namespace internal

void Pitot::DiscreteStepHelper(double t) {
  ReferenceFrame pitot_frame(wing_.frame(), pitot_params_.pos, dcm_b2actual_);

  // Calculate the static pressure.
  Vec3 pitot_pos_ned;
  pitot_frame.TransformOriginTo(environment_.ned_frame(),
                                ReferenceFrame::kPosition, &pitot_pos_ned);
  double P_stat__ = environment_.pressure(-pitot_pos_ned.z);

  // Calculate the local apparent wind at the pitot.  This takes into
  // account the freestream velocity and modifications due to kite
  // angular rates and upwash from the main wing. It also includes a
  // local pressure coefficient capturing interference from the nearby
  // mass balance tube.
  Vec3 local_apparent_wind_b;
  wing_.CalcLocalApparentWindB(
      pitot_params_.pos, pitot_params_.local_pressure_coeff +
                             pitot_sim_params_.local_pressure_coeff_offset,
      &local_apparent_wind_b);

  // Modify the local apparent wind to account for inflow from the
  // rotors.  Note that this is not incorporated in the
  // CalcLocalApparentWindB function because that function is used by
  // the rotors.
  if (pitot_sim_params_.include_rotor_inflow) {
    double freestream_vel =
        -Vec3Dot(&local_apparent_wind_b, &pitot_sim_params_.rotor_axis);
    Vec3 inflow_b;
    internal::CalcInflowFromRotorsAtPitot(environment_.air_density(),
                                          total_rotor_thrust(), freestream_vel,
                                          pitot_sim_params_, &inflow_b);
    Vec3Add(&local_apparent_wind_b, &inflow_b, &local_apparent_wind_b);
  }

  // Rotate body coordinates to pitot coordinates.
  Vec3 local_apparent_wind_p;
  Mat3Vec3Mult(&dcm_b2actual_, &local_apparent_wind_b, &local_apparent_wind_p);

  // Calculate the differential pressures.
  double P_dyn__, P_alpha__, P_beta__;
  internal::CalcFivePortPressures(
      environment_.air_density(), pitot_params_.port_angle,
      local_apparent_wind_p, &P_dyn__, &P_alpha__, &P_beta__);

  actual_P_dyn_.DiscreteUpdate(t, P_dyn__);
  actual_P_stat_.DiscreteUpdate(t, P_stat__);
  actual_P_alpha_.DiscreteUpdate(t, P_alpha__);
  actual_P_beta_.DiscreteUpdate(t, P_beta__);
}

void Pitot::UpdateSensorOutputs(SimSensorMessage *sensor_message,
                                TetherUpMessage * /*tether_up*/) const {
  sensor_message->control_input_messages_updated
      .flight_comp_sensors[fc_label_] = true;
  PitotSensor *pitot =
      &sensor_message->control_input_messages.flight_comp_sensors[fc_label_]
           .pitot;

  pitot->latency_usec = static_cast<int32_t>(1e6 * pitot_sim_params_.ts);
  pitot->speed = static_cast<float>(
      InvertCal(P_dyn(), &pitot_params_.sensors[label_].dyn_cal));
  pitot->altitude = static_cast<float>(
      InvertCal(P_stat(), &pitot_params_.sensors[label_].stat_cal));
  pitot->pitch = static_cast<float>(
      InvertCal(P_alpha(), &pitot_params_.sensors[label_].alpha_cal));
  pitot->yaw = static_cast<float>(
      InvertCal(P_beta(), &pitot_params_.sensors[label_].beta_cal));
}

void Pitot::Publish() const {
  sim_telem.pitots[label_].P_dyn = P_dyn();
  sim_telem.pitots[label_].P_stat = P_stat();
  sim_telem.pitots[label_].P_alpha = P_alpha();
  sim_telem.pitots[label_].P_beta = P_beta();
}
