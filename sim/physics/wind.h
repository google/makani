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

#ifndef SIM_PHYSICS_WIND_H_
#define SIM_PHYSICS_WIND_H_

#include <gsl/gsl_vector.h>

#include <string>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"

// Functions related to wind shear and turbulence.
//
// Many of these functions define a "mean wind frame" denoted by
// the "mw" subscript.  This should not be confused with the
// wind frame defined in sim/physics/aero_frame.h, which related to
// the apparent wind at a body.
//
// The origin of the mean wind frame lies at ground level directly
// below the ground station origin.  The axes are defined by:
//   x: Points along the predominate wind vector.
//   y: Completes a right-handed frame.
//   z: Points up.
// These are chosen to be consistent with NREL's TurbSim software
// and various IEC standards.

namespace sim {

namespace physics {

namespace wind {

double CalcWindShear(double height_agl, double ref_height_agl,
                     double ref_wind_speed, double wind_shear_exponent);

class FrozenTurbulenceWindDatabase {
  friend class FrozenTurbulenceWindDatabaseTest;

 public:
  FrozenTurbulenceWindDatabase(double t0, double y0,
                               const std::string &filename);
  ~FrozenTurbulenceWindDatabase() {
    gsl_vector_free(u_);
    gsl_vector_free(v_);
    gsl_vector_free(w_);
  }
  void CalcWind(double t, const Vec3 &pos_mw, Vec3 *wind_mw) const;

 private:
  // Time offset [s] to apply.  At t = 0, positions whose x coordinate
  // (in the mw frame) is zero will fetch the database entry given at
  // time t0_.
  double t0_;
  // Y offset [m] to apply from the center of the wind database.
  double y0_;
  // Mean wind speed [m/s] that sets the rate at which the database
  // is "advected" along the mean wind coordinate x axis.
  double mean_wind_speed_;
  // Number [#] of t, y, and z grid points.
  int32_t num_t_, num_y_, num_z_;
  // Duration [s], width [m], and height [m] of the grid defining
  // the database.
  double duration_, width_, height_;
  // Vectors storing the components of the wind velocity [m/s].  Each
  // vector has num_t * num_y * num_z elements.  The u, v, and w
  // components point along the x, y, and z axes of the mean wind
  // coordinate system.
  gsl_vector *u_;
  gsl_vector *v_;
  gsl_vector *w_;

  DISALLOW_COPY_AND_ASSIGN(FrozenTurbulenceWindDatabase);
};

enum class IecTurbulenceCategory { kA, kB, kC };

enum class IecWindTurbineClass { kI, kII, kIII };

class IecWindModel {
 public:
  IecWindModel(double hub_height_agl__, double rotor_diameter,
               double hub_wind_speed, IecTurbulenceCategory turbulence_category,
               IecWindTurbineClass wind_turbine_class);

  double hub_height_agl() const { return hub_height_agl_; }
  double ntm_sigma_1() const { return ntm_sigma_1_; }
  double ewm_sigma_1() const { return ewm_sigma_1_; }
  double etm_sigma_1() const { return etm_sigma_1_; }

  // IEC 61400-1 Design situations.

  // IEC 61400-1 Sec. 6.3.1.2 -- Normal Wind Profile Model (NWP).
  double CalcNormalWindProfile(double height_agl) const;

  // IEC 61400-1 Sec. 6.3.2.1 -- Extreme wind speed model (EWM).
  double CalcExtremeWindSpeed1YearRecurrence(double height_agl,
                                             bool is_turbulent) const;
  double CalcExtremeWindSpeed50YearRecurrence(double height_agl,
                                              bool is_turbulent) const;

  // IEC 61400-1 Sec. 6.3.2.2 -- Extreme Operating Gust (EOG).
  double CalcExtremeOperatingGust(double t, double height_agl) const;

  // IEC 61400-1 Sec. 6.3.2.4 -- Extreme direction change (EDC).
  double CalcExtremeDirectionChange(double t) const;

  // IEC 61400-1 Sec. 6.3.2.5 -- Extreme coherent gust with direction
  // change (ECD).
  double CalcExtremeCoherentGustWithDirectionChange(
      double t, double height_agl, double *wind_direction) const;

  // IEC 61400-1 Sec. 6.3.2.6 -- Extreme wind shear (EWS).
  double CalcExtremeWindShearVertical(double t, double height_agl) const;
  double CalcExtremeWindShearHorizontal(double t, double height_agl,
                                        double y) const;

 private:
  // Hub height above ground level [m].
  const double hub_height_agl_;

  // Rotor diameter [m].
  const double rotor_diameter_;

  // Nominal wind speed [m/s] at hub height.
  const double hub_wind_speed_;

  // Reference wind speed [m/s], see IEC 61400-1 Sec. 6.2.
  const double v_ref_;

  // Expected value of the turbulence intensity [#] at 15 m/s.
  const double I_ref_;

  // Longitudinal turbulence scale parameter [#].
  const double lambda_1_;

  // Normal turbulence model (NTM) standard deviation [m/s].
  const double ntm_sigma_1_;

  // Extreme wind model (EWM) standard deviation [m/s].
  const double ewm_sigma_1_;

  // Extreme turbulence model (ETM) standard deviation [m/s].
  const double etm_sigma_1_;
};

}  // namespace wind

}  // namespace physics

}  // namespace sim

#endif  // SIM_PHYSICS_WIND_H_
