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

#include "sim/models/environment.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <vector>

#include "common/c_math/filter.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "common/runfiles_dir.h"
#include "control/perch_frame.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/math/util.h"
#include "sim/physics/reference_frame.h"
#include "sim/physics/wind.h"
#include "sim/physics/wind_frame.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"

using sim::physics::wind::CalcWindShear;
using sim::physics::wind::FrozenTurbulenceWindDatabase;
using sim::physics::wind::IecWindModel;

namespace {

bool UseDrydenGusts(WindModel model) {
  return model == kWindModelDrydenTurbulence ||
         model == kWindModelDatabaseWithDrydenTurbulence;
}

}  // namespace

Environment::Environment(const IecSimParams &iec_sim_params,
                         const PhysSimParams &phys_sim_params,
                         const PhysParams &phys_params,
                         const WindSensorParams &wind_sensor_params,
                         const GroundFrameParams &ground_frame_params)
    : Model("Environment", 0.1),
      iec_sim_params_(iec_sim_params),
      phys_params_(phys_params),
      phys_sim_params_(phys_sim_params),
      wind_sensor_params_(wind_sensor_params),
      ground_z_(ground_frame_params.ground_z),
      dcm_mw2g_(),
      rng_(full_name()),
      iec_wind_model_(iec_sim_params_.hub_height_agl,
                      iec_sim_params_.rotor_diameter,
                      CalcWindShear(iec_sim_params_.hub_height_agl,
                                    phys_sim_params_.wind_shear_ref_height_agl,
                                    phys_sim_params_.wind_speed,
                                    phys_sim_params_.wind_shear_exponent),
                      sim::physics::wind::IecTurbulenceCategory::kA,
                      sim::physics::wind::IecWindTurbineClass::kI),
      wind_database_(),
      ned_frame_(),
      wind_speed_(new_discrete_state(), "wind_speed",
                  phys_sim_params_.wind_speed),
      wind_speed_target_(new_discrete_state(), "wind_speed_target",
                         phys_sim_params_.wind_speed),
      wind_speed_offsets_(),
      noise_(new_discrete_state(), "noise", 0.1, kVec3Zero),
      noise_f_z1_(new_discrete_state(), "noise_f_z1", 0.1, kVec3Zero),
      noise2_f_z1_(new_discrete_state(), "noise2_f_z1", 0.1, kVec3Zero) {
  if (phys_sim_params_.wind_model == kWindModelIec) {
    if (iec_sim_params_.load_case == kIecCaseExtremeWindSpeed50Year) {
      CHECK_GE(1e-3, fabs(0.11 - phys_sim_params_.wind_shear_exponent));
    } else {
      CHECK_GE(1e-3, fabs(0.2 - phys_sim_params_.wind_shear_exponent));
    }
  }

  CalcDcmMwToG(phys_sim_params_.wind_direction, &dcm_mw2g_);

  if (phys_sim_params_.wind_model == kWindModelDatabase ||
      phys_sim_params_.wind_model == kWindModelDatabaseWithDrydenTurbulence) {
    std::string database_path(phys_sim_params_.wind_database.name);
    CHECK(!database_path.empty()) << "No database path provided!";

    if (database_path[0] != '/') {
      database_path = RunfilesDir() + "/database/wind/" + database_path;
    }
    wind_database_.reset(new FrozenTurbulenceWindDatabase(
        phys_sim_params_.wind_database_initial_time,
        phys_sim_params_.wind_database_y_offset, database_path));
  }

  for (int32_t i = phys_sim_params_.wind_speed_update.num_updates - 1; i >= 0;
       --i) {
    const WindSpeedOffset *offset =
        &phys_sim_params_.wind_speed_update.offsets[i];
    CHECK(wind_speed_offsets_.empty() ||
          wind_speed_offsets_.top().t_update >= offset->t_update)
        << "The " << i << "th programmed wind speed update is not in order ("
        << "occurs at t = " << offset->t_update << "s when the next update"
        << " occurs at t = " << wind_speed_offsets_.top().t_update << "s)";
    wind_speed_offsets_.push(*offset);
  }

  SetupDone();
}

Environment::~Environment() {}

void Environment::CalcWind(const Vec3 &pos_g, Vec3 *wind_g,
                           Vec3 *wind_omega_g) const {
  CalcWind(pos_g, wind_g);

  // Approximate the vorticity via first difference.
  // This approach described in equation (8) on page 124 of:
  //
  //     McGrath, B.E. et al. Environment-Vehicle Interaction Modeling
  //     for Unmanned Aerial System Operations in Complex Airflow
  //     Environments.  Johns Hopkins APL Technical Digest. Vol. 31,
  //     No. 2. November 2012.
  //
  // An alternative model that requires knowing the orientation of the
  // main lifting surfaces on the wing is given by equation (7) on the
  // same page of that paper and is worth looking into.
  //
  // TODO: Reformulate vorticity calculations to the method
  // described by equation (7), for a proper fix to b/78169176.

  Vec3 tmp_wind_g;

  CalcWind({pos_g.x + kFiniteDifferenceStep, pos_g.y, pos_g.z}, &tmp_wind_g);
  double dvdx = (tmp_wind_g.y - wind_g->y) / kFiniteDifferenceStep;
  double dwdx = (tmp_wind_g.z - wind_g->z) / kFiniteDifferenceStep;

  CalcWind({pos_g.x, pos_g.y + kFiniteDifferenceStep, pos_g.z}, &tmp_wind_g);
  double dudy = (tmp_wind_g.x - wind_g->x) / kFiniteDifferenceStep;
  double dwdy = (tmp_wind_g.z - wind_g->z) / kFiniteDifferenceStep;

  CalcWind({pos_g.x, pos_g.y, pos_g.z + kFiniteDifferenceStep}, &tmp_wind_g);
  double dudz = (tmp_wind_g.x - wind_g->x) / kFiniteDifferenceStep;
  double dvdz = (tmp_wind_g.y - wind_g->y) / kFiniteDifferenceStep;

  // The solid rigid body rotation is one half of the vorticity.
  wind_omega_g->x = 0.5 * (dwdy - dvdz);
  wind_omega_g->y = 0.5 * (dudz - dwdx);
  wind_omega_g->z = 0.5 * (dvdx - dudy);
}

// The wind model is a bit awkward right now.  The Dryden model is
// meant for airplanes at a specific point, not an extended object
// with a tether.  Thus, for the Dryden model to be completely
// accurate we would have to maintain separate noise filters for each
// object.  Instead, we just use the noise calculated at the wing
// position.  A similar situation occurs for the IEC cases.  The basic
// strategy is to use Update to update the noise characteristics, but
// use Calc to determine the main component of wind speed.
void Environment::CalcWind(const Vec3 &pos_g, Vec3 *wind_g) const {
  Vec3 pos_mw;
  TransformGToMw(&pos_g, &dcm_mw2g_, ground_z_, &pos_mw);

  Vec3 wind_mw = kVec3Zero;
  switch (phys_sim_params_.wind_model) {
    case kWindModelDatabase:
    case kWindModelDatabaseWithDrydenTurbulence:
      wind_database_->CalcWind(fmax(0.0, t_z1()), pos_mw, &wind_mw);
      break;
    case kWindModelIec: {
      Vec3 mean_wind_mw;
      CalcIecMeanWind(t_z1(), pos_mw, iec_sim_params_.load_case, &mean_wind_mw);
      Vec3 wind_turbulence_mw;
      CalcIecSurrogateTurbulence(iec_sim_params_.load_case,
                                 &wind_turbulence_mw);
      Vec3Add(&mean_wind_mw, &wind_turbulence_mw, &wind_mw);
      break;
    }
    case kWindModelDrydenTurbulence:
    case kWindModelNoTurbulence: {
      double wind_speed = CalcWindShear(
          pos_mw.z, phys_sim_params_.wind_shear_ref_height_agl,
          wind_speed_.val(), phys_sim_params_.wind_shear_exponent);
      double wind_azimuth =
          Crossfade(0.0, phys_sim_params_.wind_veer, pos_mw.z,
                    phys_sim_params_.wind_veer_start_height_agl,
                    phys_sim_params_.wind_veer_end_height_agl);

      wind_mw.x =
          wind_speed * cos(phys_sim_params_.wind_elevation) * cos(wind_azimuth);
      wind_mw.y = -wind_speed * cos(phys_sim_params_.wind_elevation) *
                  sin(wind_azimuth);
      wind_mw.z = -wind_speed * sin(phys_sim_params_.wind_elevation);
      break;
    }
    case kWindModelForceSigned:
    case kNumWindModels:
    default:
      LOG(FATAL) << "Invalid WindModel value: "
                 << static_cast<int32_t>(phys_sim_params_.wind_model);
      break;
  }

  if (UseDrydenGusts(phys_sim_params_.wind_model)) {
    Vec3 wind_turbulence_mw;
    CalcDrydenTurbulence(0.0, &wind_turbulence_mw);
    Vec3Add(&wind_mw, &wind_turbulence_mw, &wind_mw);
  }

  Mat3Vec3Mult(&dcm_mw2g_, &wind_mw, wind_g);
}

void Environment::UpdateWindSpeed(double t) {
  double wind_speed__ = wind_speed_.val();
  while (!wind_speed_offsets_.empty() &&
         wind_speed_offsets_.top().t_update <= t) {
    wind_speed_target_.DiscreteUpdate(
        t, phys_sim_params_.wind_speed + wind_speed_offsets_.top().offset);
    wind_speed_offsets_.pop();
  }
  RateLimit(
      wind_speed_target_.val(), -phys_sim_params_.wind_speed_update_rate_limit,
      phys_sim_params_.wind_speed_update_rate_limit, this->ts_, &wind_speed__);
  wind_speed_.DiscreteUpdate(t, wind_speed__);
}

void Environment::DiscreteStepHelper(double t) {
  if (phys_sim_params_.wind_model == kWindModelIec) {
    Vec3 noise__ = {rng_.GetNormal(), rng_.GetNormal(), rng_.GetNormal()};
    noise_.DiscreteUpdate(t, noise__);
    noise_f_z1_.DiscreteUpdate(t, kVec3Zero);
    noise2_f_z1_.DiscreteUpdate(t, kVec3Zero);
  } else if (UseDrydenGusts(phys_sim_params_.wind_model)) {
    // TODO: Input height_agl and v_app_norm.
    UpdateDrydenTurbulence(t, 0.0, 0.0);
  } else {
    noise_.DiscreteUpdate(t, kVec3Zero);
    noise_f_z1_.DiscreteUpdate(t, kVec3Zero);
    noise2_f_z1_.DiscreteUpdate(t, kVec3Zero);
  }

  UpdateWindSpeed(t);
}

// Dryden gust model.

void Environment::CalcDrydenTurbulence(double height_agl,
                                       Vec3 *wind_turbulence_mw) const {
  // TODO: Move to a unit conversion and physical
  // constants header file.  See b/13365833.
  const double m_to_ft = 3.2808;
  Vec3 sigma_ft;
  double wind_20ft =
      CalcWindShear(20.0 / m_to_ft, phys_sim_params_.wind_shear_ref_height_agl,
                    wind_speed_.val(), phys_sim_params_.wind_shear_exponent);
  CalcDrydenTurbulenceIntensities(m_to_ft * height_agl, wind_20ft * m_to_ft,
                                  &sigma_ft);
  wind_turbulence_mw->x = noise_f_z1().x * sigma_ft.x / m_to_ft;
  wind_turbulence_mw->y = noise2_f_z1().y * sigma_ft.y / m_to_ft;
  wind_turbulence_mw->z = noise2_f_z1().z * sigma_ft.z / m_to_ft;
}

// h is the altitude in meters about ground level (positive is up).
// TODO: Replace with LPFs.
void Environment::UpdateDrydenTurbulence(double t, double height_agl,
                                         double v_rel) {
  const double m_to_ft = 3.2808;
  double v_rel_ft_per_s = fmax(m_to_ft * v_rel, 1.0);
  double ts = this->ts_;

  Vec3 L_ft, tc;
  CalcDrydenTurbulenceLengthScale(m_to_ft * height_agl, &L_ft);
  Vec3Scale(&L_ft, 1.0 / v_rel_ft_per_s, &tc);

  Vec3 noise__ = {rng_.GetNormal(), rng_.GetNormal(), rng_.GetNormal()};
  Vec3Scale(&noise__, sqrt(M_PI / noise_.ts()), &noise__);

  // Turbulence transfer function (from MIL-F-8785C)
  Vec3 dnoise = {(sqrt(2.0 / M_PI * tc.x) * noise__.x - noise_f_z1().x) / tc.x,
                 (sqrt(1.0 / M_PI * tc.y) * noise__.y - noise_f_z1().y) / tc.y,
                 (sqrt(1.0 / M_PI * tc.z) * noise__.z - noise_f_z1().z) / tc.z};

  Vec3 noise_f_z1__;
  Vec3Add(&noise_f_z1(), Vec3Scale(&dnoise, ts, &noise_f_z1__), &noise_f_z1__);

  Vec3 noise2_f_z1__ = {
      ts * (noise_f_z1__.x + sqrt(3.0) * dnoise.x * tc.x - noise2_f_z1().x) /
          tc.x,
      ts * (noise_f_z1__.y + sqrt(3.0) * dnoise.y * tc.y - noise2_f_z1().y) /
          tc.y,
      ts * (noise_f_z1__.z + sqrt(3.0) * dnoise.z * tc.z - noise2_f_z1().z) /
          tc.z};
  Vec3Add(&noise2_f_z1(), &noise2_f_z1__, &noise2_f_z1__);

  noise_.DiscreteUpdate(t, noise__);
  noise_f_z1_.DiscreteUpdate(t, noise_f_z1__);
  noise2_f_z1_.DiscreteUpdate(t, noise2_f_z1__);
}

// height_agl_ft is the altitude in feet about ground level (positive is up).
void Environment::CalcDrydenTurbulenceLengthScale(double height_agl_ft,
                                                  Vec3 *length_scale) const {
  // Low altitude (MIL-F-8785C)
  double L_wg = Saturate(height_agl_ft, 10.0, 1000.0);
  double L_uvg = L_wg / pow(0.177 + 0.000823 * L_wg, 1.2);

  length_scale->x = L_uvg;
  length_scale->y = L_uvg;
  length_scale->z = L_wg;
}

// height_agl_ft is the altitude in feet about ground level (positive is up).
void Environment::CalcDrydenTurbulenceIntensities(double height_agl_ft,
                                                  double wind20_ft,
                                                  Vec3 *sigma) const {
  // Low altitude (MIL-F-8785C)
  double height_agl_sat_ft = Saturate(height_agl_ft, 0.0, 1000.0);
  double sigma_wg = 0.1 * wind20_ft;
  double sigma_uvg = sigma_wg / pow(0.177 + 0.000823 * height_agl_sat_ft, 0.4);

  sigma->x = sigma_uvg;
  sigma->y = sigma_uvg;
  sigma->z = sigma_wg;
}

// IEC 61400-1 Design situations.

void Environment::CalcIecMeanWind(double t, const Vec3 &pos_mw,
                                  IecDesignLoadCase iec_case,
                                  Vec3 *mean_wind_mw) const {
  const double t_start = iec_sim_params_.event_t_start;
  double wind_speed;
  // Zero change in wind direction results in the wind pointing along
  // the mw-frame x axis.  A positive change in wind direction
  // rotates the the wind to point along the negative mw-frame y axis.
  double wind_direction_change = 0.0;
  // TODO: Add IEC cases for turbulent extreme winds.
  switch (iec_case) {
    case kIecCaseNormalWindProfile:
      wind_speed = iec_wind_model_.CalcNormalWindProfile(pos_mw.z);
      break;

    case kIecCaseNormalTurbulenceModel:
      // Note: It's not clear in the IEC document whether the Normal
      // Turbulence Model also includes the Normal Wind Profile.  For
      // the moment, we assume that it does not, and that the wind is
      // constant at the wind sensor level (wind_g).
      wind_speed = iec_wind_model_.CalcNormalWindProfile(
          iec_wind_model_.hub_height_agl());

      break;

    case kIecCaseExtremeWindSpeed1Year:
      wind_speed =
          iec_wind_model_.CalcExtremeWindSpeed1YearRecurrence(pos_mw.z, false);
      break;

    case kIecCaseExtremeWindSpeed50Year:
      wind_speed =
          iec_wind_model_.CalcExtremeWindSpeed50YearRecurrence(pos_mw.z, false);
      break;

    case kIecCaseExtremeOperatingGust:
      wind_speed =
          iec_wind_model_.CalcExtremeOperatingGust(t - t_start, pos_mw.z);
      break;

    case kIecCaseExtremeTurbulenceModel:
      wind_speed = iec_wind_model_.CalcNormalWindProfile(pos_mw.z);
      break;

    case kIecCaseExtremeDirectionChange:
      wind_speed = iec_wind_model_.CalcNormalWindProfile(pos_mw.z);
      wind_direction_change =
          iec_wind_model_.CalcExtremeDirectionChange(t - t_start);
      break;

    case kIecCaseExtremeCoherentGustWithDirectionChange:
      wind_speed = iec_wind_model_.CalcExtremeCoherentGustWithDirectionChange(
          t - t_start, pos_mw.z, &wind_direction_change);
      break;

    case kIecCaseExtremeWindShearVertical:
      wind_speed =
          iec_wind_model_.CalcExtremeWindShearVertical(t - t_start, pos_mw.z);
      break;

    case kIecCaseExtremeWindShearHorizontal:
      // Note that the IEC model has z-coordinates up and
      // x-coordinates pointing down-wind.  These two flips mean that
      // the IEC coordinate y points in the same direction as our
      // convention.
      wind_speed = iec_wind_model_.CalcExtremeWindShearHorizontal(
          t - t_start, pos_mw.z, pos_mw.y);
      break;

    default:
      wind_speed = 0.0;
      LOG(FATAL) << "Invalid IEC load case.";
  }

  mean_wind_mw->x = wind_speed * cos(wind_direction_change);
  mean_wind_mw->y = -wind_speed * sin(wind_direction_change);
  mean_wind_mw->z = 0.0;
}

void Environment::CalcIecSurrogateTurbulence(IecDesignLoadCase iec_case,
                                             Vec3 *wind_turbulence_mw) const {
  double sigma_1;
  switch (iec_case) {
    case kIecCaseNormalTurbulenceModel:
      sigma_1 = iec_wind_model_.ntm_sigma_1();
      break;

    case kIecCaseExtremeWindSpeed1Year:
    case kIecCaseExtremeWindSpeed50Year:
      sigma_1 = iec_wind_model_.ewm_sigma_1();
      break;

    case kIecCaseExtremeTurbulenceModel:
      sigma_1 = iec_wind_model_.etm_sigma_1();
      break;

    default:
      LOG(FATAL) << "Invalid IEC load case.";
    case kIecCaseNormalWindProfile:
    case kIecCaseExtremeOperatingGust:
    case kIecCaseExtremeDirectionChange:
    case kIecCaseExtremeCoherentGustWithDirectionChange:
    case kIecCaseExtremeWindShearVertical:
    case kIecCaseExtremeWindShearHorizontal:
      sigma_1 = 0.0;
      break;
  }
  // These standard deviations are chosen to meet requirements in IEC
  // 61400-1 Sec. 6.3.
  Vec3 sigma = {sigma_1, 0.7 * sigma_1, 0.5 * sigma_1};
  Vec3Mult(&noise(), &sigma, wind_turbulence_mw);
}
