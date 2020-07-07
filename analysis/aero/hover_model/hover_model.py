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

"""Aerodynamic model used during hover."""

import os

import makani
from makani.analysis.aero import airfoil
from makani.analysis.aero import apparent_wind_util
from makani.analysis.aero.avl import avl_reader
from makani.analysis.aero.hover_model import wake_model
from makani.config import mconfig
from makani.control import system_types
from makani.lib.python import build_info
from makani.lib.python import c_helpers
from makani.lib.python import wing_flag
import numpy as np


def GetParams(wing_model, wing_serial, use_wake_model=True):
  """Returns the set of parameters used for the model.

  Args:
    wing_model: Wing model (e.g. 'm600').
    wing_serial: String giving the desired wing serial number (e.g. '01').
    use_wake_model: Boolean flag that determines whether the advected
        rotor wake is included in the calculation of the local
        apparent wind.

  Returns:
    Parameter structure for the hover model.
  """
  wing_config_name = wing_flag.WingModelToConfigName(wing_model)

  if wing_config_name == 'oktoberkite':
    wing_serial_helper = c_helpers.EnumHelper('WingSerialOktoberKite',
                                              system_types)
    db_name = 'avl/oktoberkite.avl'
  elif wing_config_name == 'm600':
    wing_serial_helper = c_helpers.EnumHelper('WingSerial', system_types)
    db_name = 'avl/m600_low_tail_no_winglets.avl'
  else:
    assert False, 'Wing model, %s, is not recognized.' % wing_config_name

  mconfig.WING_MODEL = wing_config_name
  system_params = mconfig.MakeParams(
      wing_config_name + '.system_params',
      overrides={
          'wing_serial': wing_serial_helper.Value(wing_serial)
      }, override_method='derived')

  # Thrust level is necessary for the propeller wake model.  It is
  # determined by assuming the kite is supporting the full weight of
  # the tether and has to balance the pitching moment about the
  # C.G. with differential thrust between the top and bottom motor
  # rows:
  #
  #   4.0 * thrust_top + 4.0 * thrust_bot = weight
  #   z_top * thrust_top + z_bot * thrust_bot = 0.0
  total_mass = (system_params['wing']['m']
                + (system_params['tether']['linear_density']
                   * system_params['tether']['length']))
  weight = system_params['phys']['g'] * total_mass

  main_wing_incidence_deg = system_params['wing']['wing_i']

  rotor_pos_z = [
      rotor['pos'][2] - system_params['wing']['center_of_mass_pos'][2]
      for rotor in system_params['rotors']
  ]

  z_bot = np.mean([z for z in rotor_pos_z if z > 0.0])
  z_top = np.mean([z for z in rotor_pos_z if z < 0.0])
  thrust_bot = weight / (4.0 * (1.0 + (-z_bot / z_top)))
  thrust_top = thrust_bot * (-z_bot / z_top)

  rotors = [{
      'pos': rotor['pos'],
      'radius': rotor['D'] / 2.0,
      'thrust': thrust_bot if rotor['pos'][2] > 0.0 else thrust_top
  } for rotor in system_params['rotors']]

  # It is assumed that all rotors have the same pitch angle.
  rotor_pitches = [
      np.arctan2(-rotor['axis'][2], rotor['axis'][0])
      for rotor in system_params['rotors']
  ]
  assert np.all(rotor_pitches == rotor_pitches[0])

  aero_home = os.path.join(makani.HOME, 'analysis/aero/')

  wing = avl_reader.AvlReader(os.path.join(aero_home, db_name))

  # TODO: Extract airfoil information from avl_reader.py.
  if wing_config_name == 'm600':
    # The maximum value of 16.0 degrees for the M600 is because of the airfoil
    # hysteresis going from attached flow to detached flow, which occures during
    # HoverAccel.
    stall_angles_deg = [1.0, 16.0]
  elif wing_config_name == 'oktoberkite':
    # (b/146071300) Currently, there is no understanding of the hysteresis
    # curve. This needs to be understood and modified once available.
    # (b/146061441) Additional ramifications for this choice.
    stall_angles_deg = [1.0, 20.0]
  else:
    assert False, 'Unknown wing model.'

  # (b/146081917) Oktoberkite will utilize the same airfoils as the M600.
  outer_wing_airfoil = airfoil.Airfoil(
      os.path.join(aero_home, 'hover_model/airfoils/oref.json'),
      stall_angles_deg=stall_angles_deg)
  inner_wing_airfoil = airfoil.Airfoil(
      os.path.join(aero_home, 'hover_model/airfoils/ref.json'),
      stall_angles_deg=stall_angles_deg)
  pylon_airfoil = airfoil.Airfoil(
      os.path.join(aero_home, 'hover_model/airfoils/mp6d.json'),
      stall_angles_deg=[-10.0, 10.0])

  airfoils = {
      'Wing (panel 0)': outer_wing_airfoil,
      'Wing (panel 1)': outer_wing_airfoil,
      'Wing (panel 2)': inner_wing_airfoil,
      'Wing (panel 3)': inner_wing_airfoil,
      'Wing (panel 4)': inner_wing_airfoil,
      'Wing (panel 5)': inner_wing_airfoil,
      'Wing (panel 6)': inner_wing_airfoil,
      'Wing (panel 7)': inner_wing_airfoil,
      'Wing (panel 8)': outer_wing_airfoil,
      'Wing (panel 9)': outer_wing_airfoil,
      'Pylon 1': pylon_airfoil,
      'Pylon 2': pylon_airfoil,
      'Pylon 3': pylon_airfoil,
      'Pylon 4': pylon_airfoil,
      'Horizontal tail': airfoil.Airfoil(
          os.path.join(aero_home, 'hover_model/airfoils/f8.json'),
          stall_angles_deg=[-10.0, 10.0]),
      'Vertical tail': airfoil.Airfoil(
          os.path.join(aero_home, 'hover_model/airfoils/approx_blade_rj8.json'),
          stall_angles_deg=[-10.0, 10.0])
  }

  # TODO: Extract flap index information from
  # avl_reader.py.
  # The indices of the wing pannel indicate which flap number they are going to
  # be in the controller, i.e. "0" is A1 for the M600 and "7" is the rudder for
  # the M600. An index of -1 means that the panel has no flap section.
  if wing_config_name == 'm600':
    delta_indices = {
        'Wing (panel 0)': 0,
        'Wing (panel 1)': 1,
        'Wing (panel 2)': -1,
        'Wing (panel 3)': 2,
        'Wing (panel 4)': -1,
        'Wing (panel 5)': -1,
        'Wing (panel 6)': 3,
        'Wing (panel 7)': -1,
        'Wing (panel 8)': 4,
        'Wing (panel 9)': 5,
        'Pylon 1': -1,
        'Pylon 2': -1,
        'Pylon 3': -1,
        'Pylon 4': -1,
        'Horizontal tail': 6,
        'Vertical tail': 7
    }
  elif wing_config_name == 'oktoberkite':
    delta_indices = {
        'Wing (panel 0)': 0,
        'Wing (panel 1)': 1,
        'Wing (panel 2)': 2,
        'Wing (panel 3)': -1,
        'Wing (panel 4)': -1,
        'Wing (panel 5)': -1,
        'Wing (panel 6)': -1,
        'Wing (panel 7)': 3,
        'Wing (panel 8)': 4,
        'Wing (panel 9)': 5,
        'Pylon 1': -1,
        'Pylon 2': -1,
        'Pylon 3': -1,
        'Pylon 4': -1,
        'Horizontal tail': 6,
        'Vertical tail': 7
    }

  # TODO: Modify the hover model to accept the
  # avl_reader.py surface dictionary directly.
  hover_model_panels = []
  for surface in wing.properties['surfaces']:
    if surface['name'] in ('Horizontal tail', 'Vertical tail', 'Pylon 1',
                           'Pylon 2', 'Pylon 3', 'Pylon 4'):
      hover_model_panels.append({
          'name': surface['name'],
          'area': surface['area'],
          'span': surface['span'],
          'surface_span': surface['span'],
          'chord': surface['standard_mean_chord'],
          'aspect_ratio': surface['span']**2.0 / surface['area'],
          # The 2.0 is a fudge factor to match the C_Y slope from AVL.
          'cl_weight': 2.0 if surface['name'][0:5] == 'Pylon' else 1.0,
          'pos_b': surface['aerodynamic_center_b'],
          'airfoil': airfoils[surface['name']],
          'incidence': surface['mean_incidence'],
          # These surfaces are single panel. So there is no relative incidence.
          'relative_incidence': 0.0,
          'dcm_b2s': surface['dcm_b2surface'],
          'delta_index': delta_indices[surface['name']]
      })
    elif surface['name'] == 'Wing':
      for i, panel in enumerate(surface['panels']):
        panel_name = surface['name'] + ' (panel %d)' % i
        hover_model_panels.append({
            'name': panel_name,
            'area': panel['area'],
            'span': panel['span'],
            'surface_span': surface['span'],
            'chord': panel['standard_mean_chord'],
            'aspect_ratio': surface['span']**2.0 / surface['area'],
            'cl_weight': (
                surface['mean_incidence'] * surface['standard_mean_chord'] /
                (panel['mean_incidence'] * panel['standard_mean_chord'])),
            'pos_b': panel['aerodynamic_center_b'],
            'airfoil': airfoils[panel_name],
            'incidence': panel['mean_incidence'],
            'relative_incidence': (panel['mean_incidence']
                                   - np.deg2rad(main_wing_incidence_deg)),
            'dcm_b2s': surface['dcm_b2surface'],
            'delta_index': delta_indices[panel_name]
        })
    else:
      assert False

  # Check that we aren't missing any panels.
  assert (len(hover_model_panels) == len(delta_indices) and
          len(hover_model_panels) == len(airfoils))

  return {
      'wing_serial': wing_serial,
      'git_commit': build_info.GetGitSha(),
      'use_wake_model': use_wake_model,
      'rotors': rotors,
      'rotor_pitch': rotor_pitches[0],
      'phys': {
          'rho': 1.1,
          'dynamic_viscosity': 1.789e-5
      },

      'wing': {
          'area': system_params['wing']['A'],
          'b': system_params['wing']['b'],
          'c': system_params['wing']['c'],
          'wing_i': system_params['wing']['wing_i']
      },

      'center_of_mass_pos': system_params['wing']['center_of_mass_pos'],

      'panels': hover_model_panels
  }


def CalcForceMomentCoeffs(alphas, betas, deltas, omega_hat, apparent_wind_speed,
                          params, local_apparent_winds_sph=None):
  """Calculates the total force moment coefficients from all of the panels.

  Args:
    alphas: Angle-of-attacks [rad] represented as a (...,) ndarray.
    betas: Sideslip angles [rad] represented as a (...,) ndarray.
    deltas: Flap deflection angles [rad] represented as a
        (num_deltas, ...,) ndarray.
    omega_hat: Non-dimensional angular rate vector [#].
    apparent_wind_speed: Apparent wind speed [m/s] of the body as a scalar.
    params: Parameters including the rotor and panel properties.
    local_apparent_winds_sph: Optional argument for a set of
        pre-computed local apparent winds for all the sampling points
        on each panel in spherical coordinates.

  Returns:
    Total force coefficient [#] from all panels as a (..., 3)
    ndarray and total moment coefficient [#] from all panels as a
    (..., 3) ndarray.
  """
  assert np.shape(alphas) == np.shape(betas) == np.shape(deltas)[1:]
  angular_rate_b = [
      omega_hat[0] * 2.0 * apparent_wind_speed / params['wing']['b'],
      omega_hat[1] * 2.0 * apparent_wind_speed / params['wing']['c'],
      omega_hat[2] * 2.0 * apparent_wind_speed / params['wing']['b']
  ]
  force_b, moment_b = _CalcTotalForceMoment(alphas, betas, deltas,
                                            apparent_wind_speed, angular_rate_b,
                                            params, local_apparent_winds_sph)
  force_normalization = (0.5 * params['phys']['rho'] * apparent_wind_speed**2.0
                         * params['wing']['area'])
  force_b /= force_normalization
  moment_b.T[0] /= force_normalization * params['wing']['b']
  moment_b.T[1] /= force_normalization * params['wing']['c']
  moment_b.T[2] /= force_normalization * params['wing']['b']

  return np.concatenate((force_b, moment_b), axis=-1)


def _CalcTotalForceMoment(alphas, betas, deltas, apparent_wind_speed,
                          angular_rate_b, params,
                          local_apparent_winds_sph=None):
  """Calculates the total force moment from all of the panels.

  Args:
    alphas: Angle-of-attacks [rad] represented as a (...,) ndarray.
    betas: Sideslip angles [rad] represented as a (...,) ndarray.
    deltas: Flap deflection angles [rad] represented as a (...,) ndarray.
    apparent_wind_speed: Apparent wind speed [m/s] of the body.
    angular_rate_b: Angular rate [rad/s] of the body.
    params: Parameters including the rotor and panel properties.
    local_apparent_winds_sph: Optional argument for a set of
        pre-computed local apparent winds for all the sampling points
        on each panel in spherical coordinates.

  Returns:
    Total force [N] from all panels as a (..., 3) ndarray and total
    moment [N-m] from all panels as a (..., 3) ndarray.
  """
  total_force_b = np.zeros(np.shape(alphas) + (3,))
  total_moment_b = np.zeros(np.shape(alphas) + (3,))

  for i, panel in enumerate(params['panels']):
    # Use 0 rad for the sections of the wing that don't have a flap
    # because they should still have the fixed flap at 0 rad.

    panel_deltas = (
        deltas[panel['delta_index']] if panel['delta_index'] >= 0 else
        0.0 if panel['name'][0:4] == 'Wing' else
        None)

    if local_apparent_winds_sph:
      # Use pre-computed values for the local apparent winds if
      # they're available.
      airspeeds_s = local_apparent_winds_sph[i]['airspeeds_s']
      alphas_s = local_apparent_winds_sph[i]['alphas_s']
      betas_s = local_apparent_winds_sph[i]['betas_s']

      # The pre-computed local apparent winds are only calculated on a
      # tensor grid of alphas and betas, and not on a grid with a
      # third dimension for the flap deflection.  For cases where
      # there is an extra dimension for the flap deflection, simply
      # tile the values.
      if len(np.shape(alphas)) == 3:
        tiling = (1, 1, 1, np.shape(alphas)[2])
        airspeeds_s = np.tile(airspeeds_s[..., np.newaxis], tiling)
        alphas_s = np.tile(alphas_s[..., np.newaxis], tiling)
        betas_s = np.tile(betas_s[..., np.newaxis], tiling)
    else:
      # Compute airspeeds, alphas, betas for each sample point on the panel.
      airspeeds_s, alphas_s, betas_s = _CalcPanelLocalApparentWindSph(
          alphas, betas, angular_rate_b, apparent_wind_speed, panel, params)

    # Calculate the downwash seen at the tail.
    if panel['name'] == 'Horizontal tail':
      downwash = _EstimateDownwash(panel['name'], alphas)
    elif panel['name'] == 'Vertical tail':
      downwash = _EstimateDownwash(panel['name'], -betas)
    else:
      downwash = 0.0

    force_b, moment_b = _CalcPanelForceMoment(airspeeds_s, alphas_s, betas_s,
                                              panel_deltas, panel, params,
                                              downwash_angles_s=downwash)

    total_force_b += force_b
    total_moment_b += np.cross(panel['pos_b'], force_b) + moment_b

  return total_force_b, total_moment_b


def _GetPanelSamplingPoints(panel, thickness_ratio=0.2, num_points=(1, 10, 2)):
  """Gets points near panel at which to sample the local apparent wind.

  Args:
    panel: Dictionary of parameters describing an aerodynamic panel.
    thickness_ratio: Ratio of influence region in z to chord of panel.
    num_points: Tuple of number of positions to sample along each dimension.

  Returns:
    Sampling points in body coordinates as a (size(num_points), 3) ndarray.
  """
  panel_x_s_table = np.linspace(-panel['chord'] * 0.75,
                                panel['chord'] * 0.25,
                                num_points[0]) if num_points[0] > 1 else [0.0]

  panel_y_s_table = np.linspace(-panel['span'] / 2.0,
                                panel['span'] / 2.0,
                                num_points[1]) if num_points[1] > 1 else [0.0]

  panel_z_s_table = np.linspace(-panel['chord'] * thickness_ratio / 2.0,
                                panel['chord'] * thickness_ratio / 2.0,
                                num_points[2]) if num_points[2] > 1 else [0.0]

  panel_x_s, panel_y_s, panel_z_s = np.meshgrid(panel_x_s_table,
                                                panel_y_s_table,
                                                panel_z_s_table, indexing='ij')

  panel_points_s = np.reshape(
      np.array([panel_x_s.T, panel_y_s.T, panel_z_s.T]).T,
      (np.size(panel_x_s), 3))

  # Rotate to body coordinates and add panel position offset.
  panel_points_b = np.tensordot(panel['dcm_b2s'].T, panel_points_s.T,
                                axes=1).T
  return panel_points_b + np.tile(panel['pos_b'], (np.size(panel_x_s), 1))


def _CalcPanelLocalApparentWindSph(alphas, betas, angular_rate_b,
                                   apparent_wind_speed, panel, params):
  """Calculates local apparent wind at each sampling point on a panel.

  Args:
    alphas: Angle-of-attacks [rad] represented as a (...,) ndarray.
    betas: Sideslip angles [rad] represented as a (...,) ndarray.
    angular_rate_b: Angular rate [rad/s] of the body.
    apparent_wind_speed: Apparent wind speed [m/s] of the body.
    panel: Dictionary of parameters describing and aerodynamic panel.
    params: Parameters including the rotor and panel properties.

  Returns:
    Tuple of airspeeds, angles-of-attack, and sideslip angles in
    panel coordinates.
  """
  # Turn-off the wake model for the main wing because we account for
  # the effect of rotor wash over the main wing separately in the
  # simulator.
  # Also turn off the wake model for the pylons because CFD studies have shown
  # the net side force over all pylons is negligible. Pylons 1 and 4 cancel
  # out pylons 2 and 3 due to top rotor rotation direction difference and
  # rotor swirl effect on pylon angle of attack. Flight tests have also
  # shown that there is less side force on the kite than expected from the
  # pylons being blown. This should be re-evaluated if the rotor spin
  # directions are changed from the double reverse rainbow.
  use_wake_model = (params['use_wake_model'] and panel['name'][0:4] != 'Wing'
                    and panel['name'][0:5] != 'Pylon')

  panel_points_b = _GetPanelSamplingPoints(panel)
  tiled_panel_points_b = np.transpose(
      np.tile(panel_points_b, alphas.shape + (1, 1)),
      [len(alphas.shape)] + list(range(len(alphas.shape))) +
      [len(alphas.shape) + 1])

  local_apparent_wind_b = _CalcLocalApparentWind(
      tiled_panel_points_b, angular_rate_b, apparent_wind_speed,
      np.tile(alphas, (panel_points_b.shape[0],) + (1,) * len(alphas.shape)),
      np.tile(betas, (panel_points_b.shape[0],) + (1,) * len(betas.shape)),
      use_wake_model, params)

  # Rotate apparent wind to panel coordinates, which are aligned
  # with body coordinates for the horizontal panels and are rotated
  # such that panel z is aligned with body y for the vertical
  # panels.
  apparent_wind_s = np.tensordot(panel['dcm_b2s'], local_apparent_wind_b.T,
                                 axes=1).T

  airspeeds_s, alphas_s, betas_s = (
      apparent_wind_util.ApparentWindCartToSph(apparent_wind_s))

  return airspeeds_s, alphas_s, betas_s


def PrecomputeLocalApparentWindSph(alphas, betas, omega_hat,
                                   apparent_wind_speed, params):
  """Calculates local apparent wind at each sampling point on each panel.

  Calculating the local apparent wind, based on a combination of the
  global apparent wind, body angular rate, and propwash, is the most
  expensive part of the database calculation.  Moreover, according to
  the set of approximations used in this model, the local apparent
  wind is independent of flap deflections, so there is no need to
  recalculate these values for each set of flap deflections.

  Args:
    alphas: Angle-of-attacks [rad] represented as a (...,) ndarray.
    betas: Sideslip angles [rad] represented as a (...,) ndarray.
    omega_hat: Non-dimensional angular rate vector [#].
    apparent_wind_speed: Apparent wind speed [m/s] of the body.
    params: Parameters including the rotor and panel properties.

  Returns:
    List of dictionaries, one for each panel, that contain local
    airspeeds, angles-of-attack, and sideslip angles, in panel
    coordinates, for each (sample point, alpha, beta) combination.
  """
  # Convert to dimensional angular rate vector.
  angular_rate_b = [
      omega_hat[0] * 2.0 * apparent_wind_speed / params['wing']['b'],
      omega_hat[1] * 2.0 * apparent_wind_speed / params['wing']['c'],
      omega_hat[2] * 2.0 * apparent_wind_speed / params['wing']['b']
  ]
  local_apparent_winds_sph = []
  for panel in params['panels']:
    airspeeds_s, alphas_s, betas_s = _CalcPanelLocalApparentWindSph(
        alphas, betas, angular_rate_b, apparent_wind_speed, panel, params)
    local_apparent_winds_sph.append({
        'airspeeds_s': airspeeds_s,
        'alphas_s': alphas_s,
        'betas_s': betas_s
    })
  return local_apparent_winds_sph


def _CalcPanelForceMoment(airspeeds_s, alphas_s, betas_s, deltas, panel,
                          params, downwash_angles_s=0.0):
  """Calculates the force and moment due to a single aerodynamic panel.

  Args:
    airspeeds_s: Airspeeds [m/s] in panel coordinates, represented
        as a (...,) ndarray.
    alphas_s: Angle-of-attacks [rad] in panel coordinates,
        represented as a (...,) ndarray.
    betas_s: Sideslip angles [rad] in panel coordinates, represented
        as a (...,) ndarray.
    deltas: Flap deflection angles [rad] represented as a (...,) ndarray,
        or None if this panel does not have a flap.
    panel: Dictionary of parameters describing and aerodynamic panel.
    params: Parameters including the rotor and panel properties.
    downwash_angles_s: Angle of flow [rad] at tail due to the main
        wing or the pylons.  This should be 0.0 for other panels.

  Returns:
    Force [N] from a single panel as a (..., 3) ndarray, and the moment [N-m]
    from a single panel as a (..., 3) ndarray. The moment returned is
    implicitly about the panel's airfoil coefficient moment location.
  """
  # The dynamic pressure is modified to correct force-moment for
  # panel sideslip.  This assumes the element is parallel to one of
  # the body axes and thus the magnitude scales with local angle
  # between flow and span-axis: cos^2(beta_i).  This is done to the
  # dynamic pressure rather than to the forces themselves because it
  # is useful for weighing the incidence angles by the effective force
  # they cause.

  # Calculate a surface  mean dynamic pressure from all sample points,
  # eliminating the sample point dimensionality.
  # Additional reference can be made to Selig AIAA 2010-7635; Eq (5).
  mean_dynamic_pressure = np.mean(0.5 * params['phys']['rho'] * airspeeds_s**2.0
                                  * np.cos(betas_s)**2.0, axis=0)
  tiled_mean_dynamic_pressure = np.tile(mean_dynamic_pressure[..., np.newaxis],
                                        (1, 1, 3))

  # Define a mean alpha angle for the entire surface by normalizing each sample
  # point's angle of attack and local dynamic pressure (i.e. Cl(alpha)_s*q_s) by
  # the mean dynamic pressure on the surface.
  mean_alphas_s = np.mean(0.5 * params['phys']['rho'] * airspeeds_s**2.0
                          * np.cos(betas_s)**2.0
                          * alphas_s, axis=0) / mean_dynamic_pressure

  # The purpose of the iterator is specifically to solve for the constant
  # induced angle of attack for the single panel surfaces, induced_alphas_s.
  # This includes the horizontal tail, the vertical tail, and the pylons.
  # In the case of the multiple panel surfaces, e.g. the main wing, the
  # iterating assuming that the panel is part of an enforced assumed elliptic
  # lift distribution.
  #
  # Max iterations before giving up on convergence is set to 100.
  it = 0
  max_it = 100
  urf = 0.4

  # Convergence tolerance is 0.05 degrees. At a Cl = 1.0 this is equivalent to
  # less than 1 percent error in the lift prediction.
  alpha_epsilon = np.deg2rad(0.05)
  converged = False
  induced_alphas_s = np.zeros(np.shape(mean_alphas_s))

  while not converged:
    # Compute individual sample point effective angle of attack, accounting for
    # induced alpha due to circulation, downwash due to an upstream 3D wing
    # circulation field, and the defined incidence angle of the panel relative
    # to the body coordinate system (recall that the geometry of the panel is
    # flattened against the body coordinate planes).
    alphas_eff_s = (mean_alphas_s - induced_alphas_s - downwash_angles_s +
                    panel['incidence'])

    if panel['name'] == 'Horizontal tail':
      # The horizontal tail is a special case because the entire tail is
      # rotated.
      cls, cds, cms = panel['airfoil'].GetCoeffs(alphas_eff_s + deltas)
    else:
      cls, cds, cms = panel['airfoil'].GetCoeffs(alphas_eff_s, delta=deltas)

    # M600: Apply the 'cl_weight' fudge factor, which for the main wing
    # converts each panel CL to an equivalent wing CL and for the
    # pylons is tuned to match AVL.  Additionally, for the main wing,
    # increase the local effect of the flap. This is a huge hack, but
    # it's the best I could do with a flat plate model.
    # OktoberKite: Apply the elliptic lift distribution model and do not use the
    # cl_weights.
    #
    # (b/145304745) Stall range breakout between wing and non-wing panels.
    if panel['name'].startswith('Wing'):
      cl0s, _, _ = panel['airfoil'].GetCoeffs(alphas_eff_s, delta=0.0)
      fudge_scale = 3.0
      if mconfig.WING_MODEL == 'm600':
        weighted_cls = (
            (cl0s + (cls - cl0s) * fudge_scale) * panel['cl_weight'])
      elif mconfig.WING_MODEL == 'oktoberkite':
        lat_yb = 2.0 * panel['pos_b'][1] / panel['surface_span']
        weighted_cls = (cl0s + (cls - cl0s) * fudge_scale) * (
            np.sqrt(1.0 - lat_yb ** 2))
      stall_range = np.asarray([25.0, 180.0])
    else:
      weighted_cls = cls * panel['cl_weight']
      stall_range = np.asarray([15.0, 180.0])

    # Apply 2d-to-3d corrections for all strip panel aero coefficients based
    # on post stall performance.
    cls, cds, cms = _Convert2dTo3d(cls, cds, cms,
                                   mean_alphas_s - induced_alphas_s,
                                   panel['aspect_ratio'], stall_range)

    # Calculate the induced angle-of-attack for surface comprised of a single
    # panel. This assumes the ideal elliptical lift distribution where induced
    # angle of attack becomes constant across the entire panel.
    alpha_ind_s = weighted_cls / (np.pi * panel['aspect_ratio'])

    # Use under-relaxation factor (urf) to predict the next iteration's induced
    # angle of attack:
    #   alpha_i^{k+1} = urf * alpha_i_predictded + (1.0 - urf) * alpha_i^{k}
    # where k is the iteration level.
    induced_alphas_s = urf * alpha_ind_s + (1.0 - urf) * induced_alphas_s

    converged = np.allclose(induced_alphas_s, alpha_ind_s, atol=alpha_epsilon)

    it += 1
    if it > max_it:
      print 'Panel %s did not converge within %i iterations.' % (
          panel['name'], max_it)
      break

  # Rotate lift and drag coefficients to body coordinates.  This also
  # accounts for the induced drag because we include the induced
  # angle-of-attack in the rotation.  Note that cy does not exist because all
  # forces and moments are only in the panel parallel and normal directions.
  cxs = (-cds * np.cos(mean_alphas_s - induced_alphas_s) +
         cls * np.sin(mean_alphas_s - induced_alphas_s))
  czs = (-cds * np.sin(mean_alphas_s - induced_alphas_s) -
         cls * np.cos(mean_alphas_s - induced_alphas_s))

  # Scale coefficients by dynamic pressure and length scales to
  # convert to forces and moments.
  zeros = np.zeros(np.shape(cxs.T))
  force_s = (tiled_mean_dynamic_pressure * panel['area'] *
             np.array([cxs.T, zeros, czs.T]).T)
  moment_s = (tiled_mean_dynamic_pressure * panel['area'] * panel['chord'] *
              np.array([zeros, cms.T, zeros]).T)

  # Rotate the panel forces and moments into the body coordinate system.
  force_b = np.tensordot(panel['dcm_b2s'].T, force_s.T, axes=1).T
  moment_b = np.tensordot(panel['dcm_b2s'].T, moment_s.T, axes=1).T

  return force_b, moment_b


# TODO: Many of the parameters in this function could be
# automatically calculated from the geometry and airfoil parameters.
def _EstimateDownwash(panel_name, alphas):
  """Estimates downwash angle at horizontal and vertical stabilizer.

  The downwash angle, eps, is calculated, following a similar
  derivation to that given in Houghton & Carpenter, Eq. 5.19, as:

          2 C_L    4     x y'^2 (x^2 + y'^2 + 2 z^2) + y'^2 (x^2 + z^2) * length
    eps = ----- * ---- * -------------------------------------------------------
          pi AR   pi^2             (x^2 + z^2) (y'^2 + z^2) * length
                 |_____________________________________________________________|
                                                |
                                            multiplier

  where x and z are the distances between the main wing or pylon and
  the tail, y is the distance between the tail and the tips of the
  main wing or pylon, and length is sqrt(x^2 + y'^2 + z^2).  Note that
  an equivalent y, which is defined by

    y' = pi/4 * b/2,

  is used to model the simplified, constant circulation, horseshoe
  vortex (see Houghton & Carpenter 5.3).

  Args:
    panel_name: Name of the panel.
    alphas: Incidence angle [rad] of wing or pylon.

  Returns:
    Downwash angle [rad].
  """
  if panel_name == 'Horizontal tail':
    # Aspect ratio [#] and lift coefficient [#] of the main wing at
    # zero angle-of-attack.
    aspect_ratio = 20.0
    panel_cl0 = 2.05

    # High and low stall angles [rad] of the main wing.
    high_stall_angle = 0.1
    low_stall_angle = -0.3

    # Multiplier based on a simplified horseshoe vortex model (see
    # above).
    multiplier = 1.12

  elif panel_name == 'Vertical tail':
    # Aspect ratio [#] and lift coefficient [#] of a single pylon,
    # referenced to pylon area, at zero sideslip.
    aspect_ratio = 2.4
    panel_cl0 = -0.25

    # High and low stall angles [rad] of the pylons.  Note that the
    # incidence angle is for the vertical stabilizer, so the high and
    # low are inverted for the pylons.
    high_stall_angle = 0.2
    low_stall_angle = -0.2

    # Multiplier based on a simplified horseshoe vortex model (see
    # above).  This includes all four pylons, though is predominately
    # the result of the inner pylons.
    multiplier = 1.06

  else:
    return 0.0

  # Adjust lift slope based on aspect ratio.  See Anderson, Aircraft
  # Performance and Design, Eq. 2.15 with a0 = 2 * pi and e = 1.
  dcl_dalpha = 2.0 * np.pi * aspect_ratio / (2.0 + aspect_ratio)

  # Calculate the unstalled lift coefficient.
  panel_cl_unstalled = dcl_dalpha * alphas + panel_cl0

  # Calculate the stalled lift coefficient [Fluid Dynamic Lift,
  # Ch. 4-23, Eq. 16].
  panel_cl_stalled = 2.0 * np.sin(alphas) * np.cos(alphas)

  # Blend the unstalled and stalled lift coefficients based on the
  # incidence angle.
  weight = (np.clip((high_stall_angle - alphas) / 0.05, -0.5, 0.5) +
            np.clip((alphas - low_stall_angle) / 0.05, -0.5, 0.5))
  panel_cl = (weight * panel_cl_unstalled +
              (1.0 - weight) * panel_cl_stalled)

  return multiplier * 2.0 * panel_cl / (np.pi * aspect_ratio)


def _CalcLocalApparentWind(position_b, angular_rate_b, apparent_wind_speed,
                           alphas, betas, use_wake_model, params):
  """Calculates local apparent wind vector in body coordinates.

  The local apparent wind is composed of three components: the body
  apparent wind at the reference center, the apparent wind due to the
  rotation of the body, and the wake from the propellers.

  Args:
    position_b: Position [m] in body coordinates, represented as a
        (..., 3) ndarray, at which to calculate the local apparent wind.
    angular_rate_b: Angular rate [rad/s] of wing, represented as a
        (..., 3) ndarray.
    apparent_wind_speed: Apparent wind speed [m/s].
    alphas: Angle-of-attacks [rad], represented as a (..., 3) ndarray.
    betas: Sideslip angles [rad], represented as a (..., 3) ndarray.
    use_wake_model: Add velocity increment from the rotor wake model if true.
    params: Parameters such as the rotor positions, thrusts, etc.

  Returns:
    Local apparent wind vector [m] in body coordinates, represented as a
    (..., 3) ndarray.
  """
  if len(np.shape(position_b)) == 1:
    position_b = np.tile(position_b, np.shape(alphas) + (1,))
  if len(np.shape(angular_rate_b)) == 1:
    angular_rate_b = np.tile(angular_rate_b, np.shape(alphas) + (1,))
  assert (position_b.shape[:-1] == angular_rate_b.shape[:-1]
          == alphas.shape == betas.shape)

  apparent_wind_b = apparent_wind_util.ApparentWindSphToCart(
      apparent_wind_speed, alphas, betas)
  rotational_vel_b = np.cross(angular_rate_b, position_b)
  if use_wake_model:
    wake_vel_b = wake_model.SumWakeVelocityIncrementsAtPoint(
        position_b, apparent_wind_b, params)
  else:
    wake_vel_b = np.zeros(np.shape(apparent_wind_b))

  return apparent_wind_b - rotational_vel_b + wake_vel_b


def _Convert2dTo3d(cls_2d, cds_2d, cms_2d, alphas_s, aspect_ratio_s,
                   stall_range):
  """Converts aero coefficients from 2d to 3d for stalled sections.

  Airfoil coefficients (2d) are converted to 3d wing coefficients
  based on aspect ratio of the wing/panel and the local angle of
  attack. The approach is based on the method described in:
  Selig, 2010, "Modeling Full-Envelope Aerodynamics of Small UAVs
  in Realtime", AIAA Atmospheric Flight Mechanics 2010 Conference.
  (https://goo.gl/1vi4jK)

  Args:
    cls_2d: The two-dimensional airfoil lift coefficients [#], represented
        as a 2d ndarray with shape (num_alphas, num_betas).
    cds_2d: The two-dimensional airfoil drag coefficients [#], represented
        as a 2d ndarray with shape (num_alphas, num_betas).
    cms_2d: The two-dimensional airfoil moment coefficients [#], represented
        as a 2d ndarray with shape (num_alphas, num_betas).
    alphas_s: Angle-of-attacks [rad] of the panel, represented
        as a 2d ndarray with shape (num_alphas, num_betas).
    aspect_ratio_s: Aspect ratio [#] of the panel.
    stall_range: Lower and upper absolute value of stall alpha angle [deg] where
        the correction factor should be applied as a (2,) array.

  Returns:
    Lift (cls_3d), drag (cds_3d), and moment (cms_3d) coefficients [#]
        for the 3d panel, represented each as ndarrays of shape
        (num_alphas, num_betas).
  """
  # Set the alpha range over which the correction applies (stalled flow).
  alpha_lower = np.deg2rad(stall_range[0])
  alpha_upper = np.deg2rad(stall_range[1])

  # Calculate the cosine weighting function, w, for the correction, and
  # apply only over the alpha range, setting the correction to zero for
  # conditions where the panel is not stalled. The absolute values
  # ensure the correction also applies in the negative stall region.
  w = np.cos(np.pi * (abs(alphas_s) - alpha_lower) /
             (alpha_upper - alpha_lower) - (np.pi / 2.0))
  w *= np.multiply((abs(alphas_s) >= alpha_lower).astype(int),
                   (abs(alphas_s) <= alpha_upper).astype(int))

  # k_cd is the empirical drag coefficient scaling factor to go from a 2d
  # to 3d panel based on aspect ratio. The same scaling factor is also
  # applied to the lift and moment coefficients (ref: Selig 2010).
  k_cd = 1.0 - 0.41 * (1.0 - np.exp(-17.0 / aspect_ratio_s))
  cds_3d = cds_2d * (1.0 - w * (1.0 - k_cd))
  cls_3d = cls_2d * (1.0 - w * (1.0 - k_cd))
  cms_3d = cms_2d * (1.0 - w * (1.0 - k_cd))

  return cls_3d, cds_3d, cms_3d
