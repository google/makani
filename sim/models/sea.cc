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

#include "sim/models/sea.h"
#include "common/c_math/optim.h"
#include "sim/sim_telemetry.h"

#define SEA_REFRESH_TIME_STEP 0.2  // Seconds.

Sea::Sea(const Environment &environment, const GroundFrame &ground_frame,
         const SeaSimParams &sea_sim_params, double msl_pos_z_g)
    : Model("Sea", SEA_REFRESH_TIME_STEP),
      sea_sim_params_(sea_sim_params),
      environment_(environment),
      ground_frame_(ground_frame),
      w_i(),
      H_i(),
      c_i(),
      phi_i(),
      msl_pos_z_g_(msl_pos_z_g),
      wave_transl_coord(),
      wave_x_coord(),
      wave_y_coord(),
      wave_elev_g(),
      rng_(full_name()) {
  Init();
  SetupDone();
}

typedef struct {
  double w0;
  double wc;
  double dw0;
  int N;
} FrequencyMultiplierEquationParams;

double FrequencyMultiplierEquation(double kw, const void *context) {
  // The equation to solve is:
  //
  //                     kw^(N-1) - 1
  //    wc = w0 + dw0 * --------------
  //                        kw - 1
  //
  // Based on the solution to:
  //
  // wc = w0 + dw0 * Sum(kw^n, n=0..N-2)
  //
  // See https://www.wolframalpha.com/input/?i=sum(x%5En,+n%3D0..N-2)
  const FrequencyMultiplierEquationParams *params =
      (const FrequencyMultiplierEquationParams *)context;
  return (params->wc - params->w0) / params->dw0 -
         (pow(kw, params->N - 1) - 1) / (kw - 1);
}

double FrequencyMultiplierEquationDerivative(double kw, const void *context) {
  // This is the derivative of:
  //
  //                        kw^(N-1) - 1
  //    (wc - w0) / dw0 -  --------------
  //                            kw - 1
  //
  // with respect to kw.
  const FrequencyMultiplierEquationParams *params =
      (const FrequencyMultiplierEquationParams *)context;
  return (pow(kw, params->N - 1) - 1) / Square(kw - 1) -
         pow(kw, params->N - 2) * (params->N - 1) / (kw - 1);
}

void Sea::Init() {
  // Initializes the Sea model parameters using:
  //
  //   - Linear (Airy) wave theory. See:
  //       https://en.wikipedia.org/wiki/Airy_wave_theory
  //   - JONSWAP wave spectrum. See:
  //       https://wikiwaves.org/Ocean-Wave_Spectra#JONSWAP_Spectrum
  //   - Geometric frequency progression sampling. See:
  //       https://www.orcina.com/webhelp/OrcaFlex/Content/html/Wavetheory.htm#SpectrumDiscretisationTheory
  //   - Inifinite water depth approximation. See:
  //       https://folk.ntnu.no/oivarn/hercules_ntnu/LWTcourse/lwt_new_2000_Part_A.pdf
  //
  // For a more comprehensive description of the sea model, see:
  // https://docs.google.com/document/d/10--gfN41OCA1k3tjxwAdSkv7BPQm0VIaORuc5npkUWk
  w_i = new double[sea_sim_params_.number_of_waves]();
  H_i = new double[sea_sim_params_.number_of_waves]();
  c_i = new double[sea_sim_params_.number_of_waves]();
  phi_i = new double[sea_sim_params_.number_of_waves]();
  wave_transl_coord = new double[SEA_N_GRID_SEGMENTS]();
  wave_x_coord = new double[SEA_N_GRID_SEGMENTS]();
  wave_y_coord = new double[SEA_N_GRID_SEGMENTS]();
  wave_elev_g = new double[SEA_N_GRID_SEGMENTS]();

  // Initialize wave telemetry data.
  double grid_dl = 2.0 * sea_sim_params_.grid_half_length /
                   SEA_N_GRID_SEGMENTS;  // Grid spacing in wave direction [m].
  double grid_l_min = -sea_sim_params_.grid_half_length;
  double l;
  for (int i = 0; i < SEA_N_GRID_SEGMENTS; i++) {
    l = grid_l_min + i * grid_dl;
    wave_x_coord[i] =
        l * cos(sea_sim_params_.wave_heading_ned - ground_frame_.heading());
    wave_y_coord[i] =
        l * sin(sea_sim_params_.wave_heading_ned - ground_frame_.heading());
    wave_transl_coord[i] = l;
  }

  // If waves are not used, everything else is zero and we are done.
  if (!sea_sim_params_.use_waves) {
    DiscreteUpdate(0.0);
    return;
  }

  // Initialize the first wave.
  double dw = sea_sim_params_.initial_frequency_sampling_delta;
  w_i[0] = sea_sim_params_.initial_frequency;
  H_i[0] = sqrt(2.0 * TrapzIntegral(w_i[0] - 0.5 * dw, w_i[0] + 0.5 * dw,
                                    &Sea::JonswapSpectrum, 100));
  c_i[0] = Square(w_i[0]) / environment_.g();
  phi_i[0] = rng_.GetUniformReal() * 2.0 * PI;

  // Obtain frequency sampling multiplier for geometric sampling, given the
  // initial sampling delta, the desired number of waves and the desired initial
  // and cutoff frequencies.
  double kw;  // Frequency sampling multiplier.

  FrequencyMultiplierEquationParams freq_mult_eq_params;
  freq_mult_eq_params.N = sea_sim_params_.number_of_waves;
  freq_mult_eq_params.dw0 = sea_sim_params_.initial_frequency_sampling_delta;
  freq_mult_eq_params.w0 = sea_sim_params_.initial_frequency;
  freq_mult_eq_params.wc = sea_sim_params_.cutoff_frequency;
  kw = Newton(&FrequencyMultiplierEquation,
              &FrequencyMultiplierEquationDerivative, &freq_mult_eq_params, 1.1,
              1, 2, 1e-6, 10);

  // Compute wave parameters for each sampled frequency.
  double w_start, w_end;
  for (int i = 1; i < sea_sim_params_.number_of_waves; i++) {
    w_i[i] = w_i[i - 1] + dw;     // Central sampling frequency.
    w_start = w_i[i] - 0.5 * dw;  // Starting frequency for integration.
    w_end = w_i[i] + 0.5 * dw;    // Final frequency for integration.
    H_i[i] =
        sqrt(2.0 * TrapzIntegral(w_start, w_end, &Sea::JonswapSpectrum, 100));
    dw = dw * kw;  // Update the frequency delta.
    c_i[i] = Square(w_i[i]) / environment_.g();
    phi_i[i] = rng_.GetUniformReal() * 2.0 * PI;
  }

  DiscreteUpdate(0.0);
}

double Sea::GetSeaElevation(double t, double x, double y) const {
  // Returns sea elevation from the ground frame.
  //
  // Args:
  //  t: Time [s].
  //  x: X coordinate in ground frame [m].
  //  y: Y coordinate in ground frame [m].
  //
  // Returns:
  //  Sea elevation in ground frame [m]. Positive elevation is below the ground
  //  frame.
  double wave_height_msl = GetWaveHeight(t, x, y);
  double sea_elevation_g = msl_pos_z_g_ - wave_height_msl;
  return sea_elevation_g;
}

double Sea::GetWaveHeight(double t, double x, double y) const {
  // Returns wave height above the mean sea level.
  //
  // Wave height is computed using Airy (linear) wave theory. See:
  //   https://en.wikipedia.org/wiki/Airy_wave_theory
  //
  // Args:
  //  t: Time [s].
  //  x: X coordinate in ground frame [m].
  //  y: Y coordinate in ground frame [m].
  //
  // Returns:
  //  Wave height above the mean sea level [m]. Positive wave height is above
  //  the mean sea level.
  if (!sea_sim_params_.use_waves) {
    return (0.0);
  }

  double wave_height = 0;
  double xy =
      x * cos(sea_sim_params_.wave_heading_ned - ground_frame_.heading()) +
      y * sin(sea_sim_params_.wave_heading_ned - ground_frame_.heading());
  for (int i = 0; i < sea_sim_params_.number_of_waves; i++) {
    wave_height += H_i[i] * cos(w_i[i] * t - c_i[i] * xy + phi_i[i]);
  }
  return wave_height;
}

void Sea::Publish() const {
  memcpy(sim_telem.sea.wave_transl_coord, wave_transl_coord,
         SEA_N_GRID_SEGMENTS * sizeof(double));
  memcpy(sim_telem.sea.wave_elev_g, wave_elev_g,
         SEA_N_GRID_SEGMENTS * sizeof(double));
}

void Sea::GetWaveKinematics(double t, Vec3 *pos_g, Vec3 *velocity_g,
                            Vec3 *accel_g) const {
  // Returns wave particle kinematic at a point (x, y, z) in ground frame.
  //
  // Uses Airy (linear) wave theory and inifinite water depth. See:
  // folk.ntnu.no/oivarn/hercules_ntnu/LWTcourse/lwt_new_2000_Part_A.pdf
  //
  // Uses vertical kinematic stretching to limit the unrealistic amplification
  // of fluid particle velocity and acceleration above the MSL. See:
  // www.orcina.com/webhelp/OrcaFlex/Content/html/Waves,Kinematicstretching.htm
  //
  // Args:
  //  t: Time [s].
  //  pos_g: Position of the particle to evaluate, in ground frame [m].
  //  velocity_g: Particle velocity in ground frame [m/s].
  //  accel_g: Particle acceleration in ground frame [m/s^2].
  if (!sea_sim_params_.use_waves) {
    *velocity_g = kVec3Zero;
    *accel_g = kVec3Zero;
    return;
  }

  double wave_heading_g =
      sea_sim_params_.wave_heading_ned - ground_frame_.heading();
  double x = pos_g->x;
  double y = pos_g->y;
  double z = msl_pos_z_g_ - pos_g->z;  // Positive above the MSL.
  if (GetSeaElevation(t, x, y) > pos_g->z) {
    *velocity_g = kVec3Zero;
    *accel_g = kVec3Zero;
    return;
  }

  double xy = x * cos(wave_heading_g) + y * sin(wave_heading_g);
  double u = 0, w = 0, u_dot = 0, w_dot = 0;
  for (int i = 0; i < sea_sim_params_.number_of_waves; i++) {
    double phase = w_i[i] * t - c_i[i] * xy + phi_i[i];
    double cos_phase = cos(phase);
    double sin_phase = sin(phase);
    double exp_ci_z = z > 0.0 ? 1.0 : exp(c_i[i] * z);  // Vertical stretching.

    u += w_i[i] * H_i[i] * exp_ci_z * cos_phase;
    w -= w_i[i] * H_i[i] * exp_ci_z * sin_phase;
    u_dot -= Square(w_i[i]) * H_i[i] * exp_ci_z * sin_phase;
    w_dot -= Square(w_i[i]) * H_i[i] * exp_ci_z * cos_phase;
  }

  velocity_g->x = u * cos(wave_heading_g);
  velocity_g->y = u * sin(wave_heading_g);
  velocity_g->z = -w;

  accel_g->x = u_dot * cos(wave_heading_g);
  accel_g->y = u_dot * sin(wave_heading_g);
  accel_g->z = -w_dot;
}

void Sea::DiscreteStepHelper(double t) {
  // Update sea for use in telemetry and visualization only. The buoy model
  // calls GetSeaHeight() directly.
  // Note: this function is called every SEA_REFRESH_TIME_STEP seconds.
  for (int i = 0; i < SEA_N_GRID_SEGMENTS; i++) {
    wave_elev_g[i] = GetSeaElevation(t, wave_x_coord[i], wave_y_coord[i]);
  }
}

void Sea::UpdateStepHelper(double /*t*/) {}

void Sea::CalcDerivHelper(double /*t*/) {}

double Sea::TrapzIntegral(double x_start, double x_end,
                          double (Sea::*f)(double), int n_samples) {
  // Trapezoidal integral approximation.
  //
  // Args:
  //  x_start: Lower integration bound.
  //  x_end: Upper integration bound.
  //  f: Pointer to class member function to be integrated.
  //  n_samples: Number of function discretizations.
  //
  // Returns:
  //  Trapezoidal integral approximation of the function f.
  double x_spacing, x_sample, integral;
  x_spacing = (x_end - x_start) / n_samples;
  integral = (this->*f)(x_start) + (this->*f)(x_end);
  for (int j = 0; j < n_samples; j++) {
    x_sample = x_start + j * x_spacing;
    integral += 2.0 * (this->*f)(x_sample);
  }
  return (x_spacing / 2.0) * integral;
}

double Sea::JonswapSpectrum(double w) {
  // Args:
  //   w: frequency [rad/s].
  //
  // Returns:
  //   JONSWAP spectrum for w in m^2/(rad/s)
  //
  // References
  //   https://www.nrel.gov/docs/fy08osti/41958.pdf
  //   http://www.homepages.ucl.ac.uk/~uceseug/Fluids2/Wind_Turbines/Codes_and_Manuals/BS_EN_61400-3_2009.pdf
  double tp = sea_sim_params_.peak_period;
  double hs = sea_sim_params_.significant_height;
  double gamma = sea_sim_params_.gamma;
  double wp;  // Peak frequency [rad/s]
  wp = 2.0 * PI / tp;

  double sigma_w;  // Spectral width parameter.
  sigma_w = w < wp ? 0.07 : 0.09;

  if (gamma <= 0) {
    if (tp / sqrt(hs) <= 3.6) {
      gamma = 5;
    } else if (tp / sqrt(hs) <= 5) {
      gamma = exp(5.75 - 1.15 * tp / sqrt(hs));
    } else {
      gamma = 1.0;
    }
  }

  double Sj;  // JONSWAP spectrum at frequency w.
  Sj = (1.0 - 0.287 * log(gamma)) * 5.0 / 16.0 * Square(hs) *
       FourthPower(wp / w) / w * exp(-5.0 / 4.0 * FourthPower(wp / w)) *
       pow(gamma, exp(-0.5 * (Square(w - wp) / Square(sigma_w * wp))));

  return Sj;
}
