# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Aerodynamic parameters used in the simulation."""

from makani.config import mconfig
from makani.sim import sim_types
import numpy as np


@mconfig.Config(deps={
    'aero_models': 'oktoberkite.sim.aero_models',
})
def MakeParams(params):
  """Makes configuration specific aerodynamic parameters for the C-Sim.

  Indicates which low and high incidence databases to utilize.

  Args:
    params: dict containing keys for low and high incidence model databases.

  Returns:
    dict containing various aerodynamic empirical corrections and models for the
    kite.
  """
  # Select small and large deflection aerodynamic database model.
  small_deflection_aero_model = (
      params['aero_models']['small_deflection_aero_model'])
  large_deflection_aero_model = (
      params['aero_models']['large_deflection_aero_model'])

  # Default set of small deflection aerodynamic databases for each
  # model (e.g. ASWING, AVL, and VSAERO).
  default_small_deflection_databases = {
      'aswing': [
          'oktoberkite/big_m600_r05_aswing_baseline.json'
      ],
      'aswing_zero_angular_rate': [
          'oktoberkite/big_m600_r05_aswing_baseline_zero_angular_rate.json'
      ],
  }

  # Default set of large deflection aerodynamic databases for each
  # model (e.g., flat plate model with and without rotor wake model).
  default_large_deflection_databases = {
      'flat plate': [
          'oktoberkite/hover_model_okt_wind_10_no_wake_model.json'
      ],
      'flat plate with rotor wake': [
          'oktoberkite/hover_model_okt_wind_%s.json' % wind for wind
          in ['1', '1.6', '2.5', '4', '6.3', '10', '16', '25', '40', '63']
      ]
  }

  # Default table of scaling parameters capturing the effect of spoilers.
  spoiler_offset_database = 'm600/spoiler_offset_tables.json'

  # Select appropriate databases.
  small_deflection_databases = (
      default_small_deflection_databases[small_deflection_aero_model])
  large_deflection_databases = (
      default_large_deflection_databases[large_deflection_aero_model])

  five_deg = np.deg2rad(5.0)

  return {
      # Boolean describing whether the low and high incidence
      # databases should be merged [True] or whether only the low
      # incidence database should be used [False].
      'merge_databases': True,

      # Boolean describing whether both databases should be
      # calculated.  For simulation speed, it is nice to only
      # calculate whichever database will be used.  For examining the
      # databases, it is nice to calculate both.
      'force_use_both_databases': False,

      # Boolean describing whether non-linear flaps should be modeled
      # in the low incidence database. Currently only adjusts rudder.
      'use_nonlinear_flaps': True,

      # Extra pitching moment [Nm] to apply to the kite at alpha = +90 deg.
      # (see b/145632077)
      'empirical_pitching_moment_correction': 0.0,

      # Boolean describing whether spoilers (A4 and A5) should be modeled
      # in the low incidence database.
      'use_spoilers': True,

      # Rudder effectiveness corrections for the non-linear flap model
      # (see b/145632077).
      'positive_rudder_deflection_scaling': 0.4692,
      'positive_rudder_deflection_scaling_threshold': np.deg2rad(10.0),
      'negative_rudder_deflection_scaling': 0.5374,
      'negative_rudder_deflection_scaling_threshold': -np.deg2rad(10.0),

      # See selection above.
      'small_deflection_databases': [
          {'name': (small_deflection_databases[i]  # pylint: disable=g-complex-comprehension
                    if i < len(small_deflection_databases)
                    else '')}
          for i in range(sim_types.MAX_SMALL_DEFLECTION_DATABASES)
      ],

      # See selection above.
      'large_deflection_databases': [
          {'name': (large_deflection_databases[i]  # pylint: disable=g-complex-comprehension
                    if i < len(large_deflection_databases)
                    else '')}
          for i in range(sim_types.MAX_LARGE_DEFLECTION_DATABASES)
      ],
      'spoiler_offset_database': {'name': spoiler_offset_database},

      # Scale parameters for the AVL aerodynamic database.  Note that
      # the force scale factors are given in the wind frame, so that
      # drag forces act along the negative x-axis and lift forces on
      # the negative z-axis.  The moment scale factors are given in
      # the body frame.  Also note that, despite there being eight
      # parameters for each control derivative, whether all eight are
      # used depends on whether we are using an eight flap or five
      # flap aerodynamic database.
      'force_coeff_w_scale_factors': {
          'coeff': [1.0, 1.0, 1.0],
          'rate_derivatives': {
              'p': [1.0, 1.0, 1.0],
              'q': [1.0, 1.0, 1.0],
              'r': [1.0, 1.0, 1.0]
          },
          'flap_derivatives': [
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
          ]
      },
      'moment_coeff_b_scale_factors': {
          'coeff': [1.0, 1.0, 1.0],
          'rate_derivatives': {
              'p': [1.0, 1.0, 1.0],
              'q': [1.0, 1.0, 1.0],
              'r': [1.0, 1.0, 1.0]
          },
          'flap_derivatives': [
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, 1.0],
          ]
      },

      'coeff_offsets': {
          # Extra drag coefficient [#].  Drag offset to account for parasitic
          # drag components on the kite. Includes bridle drag.
          # Estimate is from BigM600 config sheet for r07c_v01.
          # https://docs.google.com/spreadsheets/d/18eGSelEsldi6UuKKpqSIw5j9_UU21-UJzTv24IcN6oE
          # This estimate is based on a simple bottom up drag estimate.
          # Many items are assumed to be the same as the M600, so the larger
          # wing area results in a smaller drag coefficient.
          'CD': 0.005,

          # Extra side-force coefficient [#].
          'CC': 0.0,

          # Extra lift coefficient [#] (i.e., CL_0 offset).
          # TODO: Do we add conservatism here, given M600 results?
          'CL': 0.0,

          # Extra roll moment coefficient [#].
          'Cl': 0.0,

          # Extra pitch moment coefficient [#].
          'Cm': 0.0,

          # Extra yaw moment coefficient [#].
          'Cn': 0.0,

          # Additional slope [#/rad] for how the roll moment coefficient
          # varies as a function of sideslip angle.
          'dCldbeta': 0.0,

          # Additional slope [#/rad] for how the pitch moment
          # coefficient varies as a function of angle-of-attack.
          # Note that there is also a potential modification of dCm/dalpha from
          # the propellers. Depending on airspeed and rotor speed, this could
          # be an addition 0.0 to 0.3, which we do not include here.
          #
          # Approximate dCm/dalpha values at zero angle-of-attack from
          # different databases:
          #
          #   ASWING (baseline_zero_angular_rate): -2.12
          #          (stage3_zero_angular_rate): -1.45
          #   VSAERO (nominal) : -2.004
          #   VSAERO (zero angular rate): -1.972
          #   AVL (nominal): -2.53
          #   AVL (zero angular rate): -2.49
          #
          'dCmdalpha': (
              0.5 if small_deflection_aero_model == 'avl' else
              0.0),

          # Additional slope [#/rad] for how the yaw moment coefficient
          # varies as a function of sideslip angle.
          'dCndbeta': 0.0,
      },

      # Offsets to apply to the flap angles.
      'flap_offsets': [0.0 for _ in range(sim_types.kNumFlaps)],

      # Angles [rad] at which the aerodynamic database transitions from the
      # low-incidence aero database to the high-incidence aero database.  The
      # transition occurs smoothly over a range specified below with the
      # linear_to_stalled_blending_angle.
      # The low and high beta_stall_angle is chosen high enough such that the
      # high-incidence model is not used in normal crosswind forward flight.
      # This is due to the high-incidence model not being trustworthy in
      # forward flight, specifically in rudder effectiveness.  See b/79981680.
      # The CFD data suggests that the kite with no slats stalls near
      # 10 deg alpha, b/116798525.

      # TODO: Get Oktoberkite values.
      'low_alpha_stall_angle': -np.deg2rad(20.0),
      'high_alpha_stall_angle': np.deg2rad(5.0),
      'low_beta_stall_angle': -np.deg2rad(20.0),
      'high_beta_stall_angle': np.deg2rad(20.0),

      # Angle [rad] over which to blend the low and high incidence
      # databases.  The blending range is centered at the relevant
      # stall angle.
      # TODO: Get Oktoberkite values.
      'linear_to_stalled_blending_angle': np.deg2rad(10.0),

      # Hinge moment coefficients with units [1, 1/deg, 1/deg, 1/deg^2]
      # obtained from 2D MSES, XFOIL and CFD.
      # Matlab scripts and reports on Gerrit at
      # makani/experimental/aerodynamics/HingeMoments/ (build f720e3f).
      # All coefficients are computed for the 8-flap areas, a reference
      # area of 32.9 m^2 and reference length of 1.28 m.

      # TODO: Get Oktoberkite values.
      'hinge_moment_coeffs': [{
          'c': -2.2881e-4,
          'c_deltad': -9.7592e-6,
          'c_alphad': -1.6304e-5,
          'c_alphad_deltad': -1.0560e-7
      }, {
          'c': -3.0449e-4,
          'c_deltad': -8.1135e-6,
          'c_alphad': -1.6464e-5,
          'c_alphad_deltad': -7.1539e-8
      }, {
          'c': -2.9219e-4,
          'c_deltad': -7.2459e-6,
          'c_alphad': -1.5198e-5,
          'c_alphad_deltad': -5.2550e-8
      }, {
          'c': -2.9219e-4,
          'c_deltad': -7.2459e-6,
          'c_alphad': -1.5198e-5,
          'c_alphad_deltad': -5.2550e-8
      }, {
          'c': -3.0449e-4,
          'c_deltad': -8.1135e-6,
          'c_alphad': -1.6464e-5,
          'c_alphad_deltad': -7.1539e-8
      }, {
          'c': -2.2881e-4,
          'c_deltad': -9.7592e-6,
          'c_alphad': -1.6304e-5,
          'c_alphad_deltad': -1.0560e-7
      }, {
          'c': -2.2721e-4,
          'c_deltad': -8.7809e-5,
          'c_alphad': -1.4299e-4,
          'c_alphad_deltad': -2.8946e-6
      }, {
          'c': -3.3485e-4,
          'c_deltad': -1.2104e-4,
          'c_alphad': 4.8421e-5,
          'c_alphad_deltad': 2.6532e-9
      }],

      # Minimum and maximum valid flap angles [rad] for AVL.  Below or
      # above these values we expect nonlinear effects to take over.
      # TODO: Get Oktoberkite values.
      'min_avl_flap_angles': [-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5],
      'max_avl_flap_angles': [five_deg, five_deg, five_deg, five_deg,
                              five_deg, five_deg, 0.5, 0.5],
  }
