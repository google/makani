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

#include "sim/physics/wind.h"

#include <glog/logging.h>
#include <hdf5.h>
#include <math.h>
#include <stdint.h>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "sim/math/util.h"

namespace sim {

namespace physics {

namespace wind {

// All references to IEC 61400-1 here refer to the 3rd edition,
// reference number IEC 61400-1:2005(E).

double CalcWindShear(double height_agl, double ref_height_agl,
                     double ref_wind_speed, double wind_shear_exponent) {
  // Note: We reference everything to ground level.
  // Note: height_agl is limited to 0.1 * height_agl_0 to avoid
  // discontinuous behavior near 0.
  return ref_wind_speed *
         pow(fmax(height_agl / ref_height_agl, 0.1), wind_shear_exponent);
}

namespace {

// Return the native HDF5 type associated with a given type.
template <typename T>
hid_t GetH5NativeType();

template <>
hid_t GetH5NativeType<double>() {
  return H5T_NATIVE_DOUBLE;
}

template <>
hid_t GetH5NativeType<int32_t>() {
  return H5T_NATIVE_INT32;
}

// Load a rank one dataset from a given file.
//
// Args:
//   file_id: HDF5 file.
//   name: The dataset to open should be located at /<name> and be rank one.
//       The type of the dataset should be match GetH5NativeType<T>().
//   num_el: Length of the dataset.
//   T: Destination pointer to write to (should be able to store num_el
//       elements).
template <typename T>
void LoadDataset(hid_t file_id, const std::string &name, int32_t num_el,
                 T *dest) {
  hid_t dataset_id = H5Dopen2(file_id, name.c_str(), H5P_DEFAULT);
  CHECK_LE(0, dataset_id);

  hid_t dataspace_id = H5Dget_space(dataset_id);
  CHECK_LE(0, dataspace_id);
  constexpr hsize_t kRank = 1;
  hsize_t dims[kRank];
  int32_t rank = H5Sget_simple_extent_ndims(dataspace_id);
  if (rank == 0) {
    CHECK_EQ(1, num_el);
  } else {
    CHECK_EQ(kRank, rank);
    H5Sget_simple_extent_dims(dataspace_id, dims, NULL);
    CHECK_EQ(num_el, dims[0]);
  }
  H5Sclose(dataspace_id);

  hid_t type_id = H5Dget_type(dataset_id);
  CHECK(H5Tequal(GetH5NativeType<T>(), type_id));

  hid_t target_space_id = H5Screate_simple(rank, dims, NULL);
  CHECK_LE(0, target_space_id);

  CHECK_LE(0, H5Dread(dataset_id, type_id, target_space_id, H5S_ALL,
                      H5P_DEFAULT, dest));

  H5Sclose(target_space_id);
  H5Tclose(type_id);
  H5Dclose(dataset_id);
}

}  // namespace

FrozenTurbulenceWindDatabase::FrozenTurbulenceWindDatabase(
    double t0, double y0, const std::string &filename)
    : t0_(t0),
      y0_(y0),
      mean_wind_speed_(-1.0),
      num_t_(-1),
      num_y_(-1),
      num_z_(-1),
      duration_(-1.0),
      width_(-1.0),
      height_(-1.0),
      u_(nullptr),
      v_(nullptr),
      w_(nullptr) {
  hid_t file_id = H5Fopen(filename.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
  CHECK_LE(0, file_id) << "Could not open file.";

  LoadDataset(file_id, "mean_wind_speed", 1, &mean_wind_speed_);
  CHECK_LT(0.0, mean_wind_speed_);
  LoadDataset(file_id, "num_t", 1, &num_t_);
  CHECK_LT(0.0, num_t_);
  LoadDataset(file_id, "num_y", 1, &num_y_);
  CHECK_LT(0.0, num_y_);
  LoadDataset(file_id, "num_z", 1, &num_z_);
  CHECK_LT(0.0, num_z_);
  LoadDataset(file_id, "duration", 1, &duration_);
  CHECK_LT(0.0, duration_);
  CHECK_LT(t0_, duration_);
  LoadDataset(file_id, "width", 1, &width_);
  CHECK_LT(0.0, width_);
  // Simple check that the y offset is in the center half of the wind database.
  // This doesn't guarantee that the CalcWind won't need to 'wrap' in y, which
  // might lead to sudden jumps in wind or vorticity, but should help us catch
  // instances where we'd be overly relying on the wrapping.
  CHECK_LT(fabs(y0_), width_ / 4.0);
  LoadDataset(file_id, "height", 1, &height_);
  CHECK_LT(0.0, height_);

  // TODO: Check for size overflow.
  int32_t size = num_t_ * num_y_ * num_z_;
  u_ = gsl_vector_alloc(size);
  v_ = gsl_vector_alloc(size);
  w_ = gsl_vector_alloc(size);
  CHECK(u_ != nullptr && v_ != nullptr && w_ != nullptr)
      << "Could not allocate memory for wind database.";

  LoadDataset(file_id, "u", size, u_->data);
  LoadDataset(file_id, "v", size, v_->data);
  LoadDataset(file_id, "w", size, w_->data);

  H5Fclose(file_id);
}

void FrozenTurbulenceWindDatabase::CalcWind(double t, const Vec3 &pos_mw,
                                            Vec3 *wind_mw) const {
  // Construct time coordinate.
  double t_scaled =
      Wrap((t + t0_ - pos_mw.x / mean_wind_speed_) / duration_, 0.0, 1.0);
  t_scaled *= num_t_ - 1;

  double y_scaled = Wrap((pos_mw.y + y0_) / width_ + 0.5, 0.0, 1.0);
  y_scaled *= num_y_ - 1;

  double z_scaled = Saturate(pos_mw.z / height_, 0.0, 1.0);
  z_scaled *= num_z_ - 1;

  wind_mw->x = gsl_interp3_scaled(t_scaled, y_scaled, z_scaled, u_, num_t_,
                                  num_y_, num_z_, kInterpOptionDefault);
  wind_mw->y = gsl_interp3_scaled(t_scaled, y_scaled, z_scaled, v_, num_t_,
                                  num_y_, num_z_, kInterpOptionDefault);
  wind_mw->z = gsl_interp3_scaled(t_scaled, y_scaled, z_scaled, w_, num_t_,
                                  num_y_, num_z_, kInterpOptionDefault);
}

namespace {

double CalcReferenceWindSpeed(IecWindTurbineClass wind_turbine_class) {
  // See Table 1 on page 22 of IEC 61400-1 Sec. 6.2.
  // Set reference wind speed average over 10 minutes.
  switch (wind_turbine_class) {
    default:
      LOG(FATAL) << "Invalid wind turbine class.";
    case IecWindTurbineClass::kI:
      return 50.0;
    case IecWindTurbineClass::kII:
      return 42.5;
    case IecWindTurbineClass::kIII:
      return 37.5;
  }
}

double CalcTurbulenceIntensity(IecTurbulenceCategory turbulence_category) {
  // See Table 1 on page 22 of IEC 61400-1 Sec. 6.2.
  // A designates the category for higher turbulence characteristics,
  // B designates the category for medium turbulence characteristics, and
  // C designates the category for lower turbulence characteristics.
  switch (turbulence_category) {
    default:
      LOG(FATAL) << "Invalid turbulence category.";
    case IecTurbulenceCategory::kA:
      return 0.16;
    case IecTurbulenceCategory::kB:
      return 0.14;
    case IecTurbulenceCategory::kC:
      return 0.12;
  }
}

double CalcLongitudinalTurbulenceScaleParameter(double hub_height_agl) {
  // See equation (5) on page 23 of IEC 61400-1 Sec. 6.3.
  // Set the longitudinal turbulence scale.
  if (hub_height_agl <= 60.0) {
    return 0.7 * hub_height_agl;
  } else {
    return 42.0;
  }
}

double CalcNormalTurbulenceModelSigma(double I_ref, double hub_wind_speed) {
  // See equation (11) on page 24 of IEC 61400-1 Sec. 6.3.1.3.
  const double b = 5.6;
  return I_ref * (0.75 * hub_wind_speed + b);
}

double CalcExtremeWindModelSigma(double hub_wind_speed) {
  // See equation (16) on page 26 of IEC 61400-1 Sec. 6.3.2.1.
  return 0.11 * hub_wind_speed;
}

double CalcExtremeTurbulenceModelSigma(double hub_wind_speed, double v_ref,
                                       double I_ref) {
  // Average velocity [m/s] based on a Rayleigh distribution.
  // See equation (9) on page 24 of IEC 61400-1 Sec. 6.3.1.1.
  const double v_ave = 0.2 * v_ref;
  // See equation (19) on page 27 of IEC 61400-1 Sec. 6.3.2.3.
  const double c = 2.0;  // [m/s]
  return c * I_ref *
         (0.072 * (v_ave / c + 3.0) * (hub_wind_speed / c - 4.0) + 10.0);
}

}  // namespace

IecWindModel::IecWindModel(double hub_height_agl__, double rotor_diameter,
                           double hub_wind_speed,
                           IecTurbulenceCategory turbulence_category,
                           IecWindTurbineClass wind_turbine_class)
    : hub_height_agl_(hub_height_agl__),
      rotor_diameter_(rotor_diameter),
      hub_wind_speed_(hub_wind_speed),
      v_ref_(CalcReferenceWindSpeed(wind_turbine_class)),
      I_ref_(CalcTurbulenceIntensity(turbulence_category)),
      lambda_1_(CalcLongitudinalTurbulenceScaleParameter(hub_height_agl_)),
      ntm_sigma_1_(CalcNormalTurbulenceModelSigma(I_ref_, hub_wind_speed_)),
      ewm_sigma_1_(CalcExtremeWindModelSigma(hub_wind_speed)),
      etm_sigma_1_(
          CalcExtremeTurbulenceModelSigma(v_ref_, I_ref_, hub_wind_speed_)) {
  CHECK_LT(0.0, hub_height_agl_);
  CHECK_GT(hub_height_agl_, rotor_diameter_ / 2.0);
  CHECK_LE(0.0, hub_wind_speed_);
}

double IecWindModel::CalcNormalWindProfile(double height_agl) const {
  // See equation (10) on page 24 of IEC 61400-1 Sec. 6.3.1.2.
  const double alpha = 0.2;
  return CalcWindShear(height_agl, hub_height_agl_, hub_wind_speed_, alpha);
}

double IecWindModel::CalcExtremeWindSpeed1YearRecurrence(
    double height_agl, bool is_turbulent) const {
  // See equations (13) and (15) on page 26 of IEC 61400-1 Sec. 6.3.2.1.
  return 0.8 * CalcExtremeWindSpeed50YearRecurrence(height_agl, is_turbulent);
}

double IecWindModel::CalcExtremeWindSpeed50YearRecurrence(
    double height_agl, bool is_turbulent) const {
  // See equation (14) on page 26 of IEC 61400-1 Sec. 6.3.2.1.
  double wind_speed = CalcWindShear(height_agl, hub_height_agl_, v_ref_, 0.11);
  if (is_turbulent) {
    return wind_speed;
  } else {
    // See equation (12) on page 25 of IEC 61400-1 Sec. 6.3.2.1.
    return 1.4 * wind_speed;
  }
}

double IecWindModel::CalcExtremeOperatingGust(double t,
                                              double height_agl) const {
  double e1_wind_speed = CalcExtremeWindSpeed1YearRecurrence(height_agl, false);

  // See equation (17) on page 26 of IEC 61400-1 Sec. 6.3.2.2.
  double gust_speed =
      fmin(1.35 * (e1_wind_speed - hub_wind_speed_),
           3.3 * (ntm_sigma_1_ / (1.0 + 0.1 * rotor_diameter_ / lambda_1_)));

  // See equation (18) on page 26 of IEC 61400-1 Sec. 6.3.2.2.
  double T = 10.5;  // [s]
  double wind_speed = CalcNormalWindProfile(height_agl);
  if (t >= 0.0 && t <= T) {
    wind_speed -= 0.37 * gust_speed * sin(3.0 * M_PI * t / T) *
                  (1.0 - cos(2.0 * M_PI * t / T));
  }
  return wind_speed;
}

double IecWindModel::CalcExtremeDirectionChange(double t) const {
  // TODO: Expose sign as a function argument.
  const double sign = 1.0;
  // See equation (20) on page 27 of IEC 61400-1 Sec. 6.3.2.4.
  double theta_e =
      sign * 4.0 *
      atan2(ntm_sigma_1_,
            hub_wind_speed_ * (1.0 + 0.1 * rotor_diameter_ / lambda_1_));

  // See equation (21) on page 27 of IEC 61400-1 Sec. 6.3.2.4.
  const double T = 6.0;  // [s]
  if (t < 0.0) {
    return 0.0;
  } else if (t <= T) {
    return 0.5 * theta_e * (1.0 - cos(M_PI * t / T));
  } else {
    return theta_e;
  }
}

double IecWindModel::CalcExtremeCoherentGustWithDirectionChange(
    double t, double height_agl, double *wind_direction) const {
  // TODO: Expose sign as a function argument.
  const double sign = 1.0;

  // Coherent gust speed [m/s] denoted by V_cg.
  // See equation (22) on page 28 of IEC 61400-1 Sec. 6.3.2.5.
  const double coherent_gust_speed = 15.0;

  // Direction change [rad] denoted by theta_cg.
  // See equation (24) on page 29 of IEC 61400-1 Sec. 6.3.2.5.
  double direction_change;
  if (hub_wind_speed_ < 4.0) {
    direction_change = M_PI;
  } else {
    direction_change = 4.0 * M_PI / hub_wind_speed_;
  }

  const double T = 10.0;  // Rise time [s].
  // See equation (23) on page 28 and equation (25)
  // on page 29 of IEC 61400-1 Sec. 6.3.2.5.
  double wind_speed = CalcNormalWindProfile(height_agl);
  if (t < 0.0) {
    *wind_direction = 0.0;
  } else if (t <= T) {
    *wind_direction = sign * 0.5 * direction_change * (1.0 - cos(M_PI * t / T));
    wind_speed += 0.5 * coherent_gust_speed * (1.0 - cos(M_PI * t / T));
  } else {
    *wind_direction = sign * direction_change;
    wind_speed += coherent_gust_speed;
  }

  return wind_speed;
}

double IecWindModel::CalcExtremeWindShearVertical(double t,
                                                  double height_agl) const {
  // TODO: Expose sign as a function argument.
  const double sign = 1.0;
  const double beta = 6.4;
  const double T = 12.0;  // [s]

  // See equation (26) on page 30 of IEC 61400-1 Sec. 6.3.2.6.
  double wind_speed = CalcNormalWindProfile(height_agl);
  if (t >= 0.0 && t <= T) {
    wind_speed +=
        sign * ((height_agl - hub_height_agl_) / rotor_diameter_) *
        (2.5 +
         0.2 * beta * ntm_sigma_1_ * pow(rotor_diameter_ / lambda_1_, 0.25)) *
        (1.0 - cos(2.0 * M_PI * t / T));
  }
  return wind_speed;
}

double IecWindModel::CalcExtremeWindShearHorizontal(double t, double height_agl,
                                                    double y) const {
  // TODO: Expose sign as a function argument.
  const double sign = 1.0;
  const double beta = 6.4;
  const double T = 12.0;  // [s]
  // See equation (27) on page 30 of IEC 61400-1 Sec. 6.3.2.6.
  double wind_speed = CalcNormalWindProfile(height_agl);
  if (t >= 0.0 && t <= T) {
    wind_speed +=
        sign * (y / rotor_diameter_) *
        (2.5 +
         0.2 * beta * ntm_sigma_1_ * pow(rotor_diameter_ / lambda_1_, 0.25)) *
        (1.0 - cos(2.0 * M_PI * t / T));
  }
  return wind_speed;
}

}  // namespace wind

}  // namespace physics

}  // namespace sim
