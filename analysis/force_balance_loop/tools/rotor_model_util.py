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

from __future__ import absolute_import
import math
import numpy as np
from matplotlib import pyplot as plt
from six.moves import zip

def Gen4RotorConfigBySizeFixedPitch(fixed_rotor_pitch, rotor_mach_limit=0.85,
                                    **kwargs):
  """Function that builds and returns a rotor function based on provided inputs.
  Rotor function is of the format that can be assigned to a kite config, based
  on a scaled Gen4 prop set at a specific pitch.

    Args:
      Fixed_rotor_pitch: blade pitch in deg. Must be one of the keys in
        rotor_fixed_pitch_type.
        TODO: Setup interpolation to allow any pitch angle.
    Kwargs: (Only 2 of the 3 rotor sizing params can be used, otherwise it
        would be over-defined.)
      a_rotors: Sets the total (combined) rotor size.
      n_rotors: Sets number of rotors.
      r_rotor: Sets radius of rotors.
    Returns:
      Function that can be used as the shaft_power_from_drag_power function
        in a kite config."""

  n_rotors = kwargs.get('n_rotors', None)
  r_rotor = kwargs.get('r_rotor', None)
  a_rotors = kwargs.get('a_rotors', None)
  rotor_params_n = sum([n_rotors is not None,
    r_rotor is not None,
    a_rotors is not None])
  assert(rotor_params_n == 2), (
      'rotors are over or under defined: {}'.format(rotor_params_n))
  # TODO: Make this a check via count instead
  if a_rotors is None:
    a_rotors = n_rotors * math.pi * r_rotor**2
  if r_rotor is None:
    r_rotor = np.sqrt(1. / np.pi * a_rotors / n_rotors)
  if n_rotors is None:
    n_rotors = a_rotors / (np.pi * r_rotor**2)

  # Fit coefficients to rotor tables from xrotor 2D runs from berryb's matlab
  # code, using Gen4 blade, 1.15 m radius, 1.225 kg/m^3 density,
  # and various pitch settings.
  # The CTturb_CPturb fit is Vander 2d polynomial of deg: [4 4].
  # The CTturb_J fit uses the custom log function defined below.
  # https://colab.corp.google.com/drive/1zNm2UjRKQmf7gIZxN8IuMnvKBJ3LJims#scrollTo=IIr4lpS5TYSi
  gen4rotor_pitch_mx_fit_coefficients = {
    -4.0: {
        'CTturb_CPturb_fit_coeffs':
            [[8.667, -0.9169, 0.03519, -0.0005819, 3.462e-06],
             [48.78, -5.312, 0.209, -0.003589, 2.25e-05],
             [78.68, -9.168, 0.394, -0.007408, 5.107e-05],
             [22.71, -4.036, 0.2336, -0.00552, 4.606e-05],
             [4.987, -0.7578, 0.04884, -0.001432, 1.518e-05]],
        'CTturb_J_fit_coeffs': [-0.4878, -0.1998, 0.4444],
        'cp_min': -5.0,
        'ct_max': 1.913,
        'ct_min': -0.456,
        'mach_max': 0.9,
        'mach_min': 0.3,
        'source': 'gen4_n4p_tables_2019_08_15.json',
        'v_min': 30.0
    },
    -2.0: {
        'CTturb_CPturb_fit_coeffs':
            [[3.617, -0.3438, 0.01172, -0.0001703, 8.772e-07],
             [13.34, -1.443, 0.05245, -0.0008174, 4.55e-06],
             [19.82, -2.23, 0.08924, -0.00153, 9.43e-06],
             [9.861, -1.356, 0.06655, -0.001357, 9.751e-06],
             [3.522, -0.5027, 0.02681, -0.0006237, 5.094e-06]],
        'CTturb_J_fit_coeffs': [-0.4353, -0.2085, 0.4953],
        'cp_min': -5.0,
        'ct_max': 2.203,
        'ct_min': -0.405,
        'mach_max': 0.9,
        'mach_min': 0.3,
        'source': 'gen4_n2p_tables_2019_08_15.json',
        'v_min': 30.0
    },
    0.0: {
        'CTturb_CPturb_fit_coeffs':
            [[1.494, -0.1298, 0.003953, -5.043e-05, 2.209e-07],
             [1.798, -0.259, 0.008243, -0.0001075, 4.554e-07],
             [-0.7518, 0.01936, -0.001205, 4.3e-05, -4.832e-07],
             [1.789, -0.1601, 0.004451, -8.392e-06, -6.007e-07],
             [2.283, -0.2439, 0.009282, -0.0001372, 3.602e-07]],
        'CTturb_J_fit_coeffs': [-0.391, -0.2226, 0.541],
        'cp_min': -5.0,
        'ct_max': 2.417,
        'ct_min': -0.362,
        'mach_max': 0.9,
        'mach_min': 0.3,
        'source': 'gen4_0p_tables_2019_08_15.json',
        'v_min': 30.0
    },
    2.0: {
        'CTturb_CPturb_fit_coeffs':
            [[0.4913, -0.03995, 0.001078, -1.158e-05, 3.793e-08],
             [-0.8655, 0.0003878, -0.0006563, 1.983e-05, -1.7e-07],
             [-4.927, 0.4215, -0.01524, 0.0002477, -1.502e-06],
             [0.4121, 0.07071, -0.007298, 0.0002212, -2.06e-06],
             [-1.506, 0.1737, -0.008137, 0.0001858, -1.821e-06]],
        'CTturb_J_fit_coeffs': [-0.3497, -0.2372, 0.5832],
        'cp_min': -5.0,
        'ct_max': 2.539,
        'ct_min': -0.323,
        'mach_max': 0.9,
        'mach_min': 0.3,
        'source': 'gen4_2p_tables_2019_08_15.json',
        'v_min': 30.0
    },
    4.0: {
        'CTturb_CPturb_fit_coeffs':
            [[-0.1145, 0.00855, -0.0002989, 4.773e-06, -2.865e-08],
             [-1.695, 0.07142, -0.002704, 4.286e-05, -2.448e-07],
             [-2.981, 0.2714, -0.01072, 0.0001812, -1.097e-06],
             [-0.9728, 0.1698, -0.009536, 0.0002283, -1.854e-06],
             [-0.8436, 0.1038, -0.005158, 0.0001243, -1.279e-06]],
        'CTturb_J_fit_coeffs': [-0.3116, -0.2528, 0.6222],
        'cp_min': -5.0,
        'ct_max': 2.581,
        'ct_min': -0.286,
        'mach_max': 0.9,
        'mach_min': 0.3,
        'source': 'gen4_4p_tables_2019_08_15.json',
        'v_min': 30.0
    },
    6.0: {
        'CTturb_CPturb_fit_coeffs':
            [[-0.1921, 0.01335, -0.0003808, 4.897e-06, -2.38e-08],
             [-1.871, 0.07869, -0.002585, 3.571e-05, -1.78e-07],
             [-4.56, 0.3858, -0.01335, 0.0001983, -1.064e-06],
             [-1.617, 0.2614, -0.01328, 0.0002804, -2.014e-06],
             [0.1926, -0.00823, -0.0007789, 4.894e-05, -7.628e-07]],
        'CTturb_J_fit_coeffs': [-0.2789, -0.2708, 0.6594],
        'cp_min': -5.0,
        'ct_max': 2.555,
        'ct_min': -0.254,
        'mach_max': 0.9,
        'mach_min': 0.3,
        'source': 'gen4_6p_tables_2019_08_15.json',
        'v_min': 30.0
    },
    8.0: {
        'CTturb_CPturb_fit_coeffs':
            [[-0.13, 0.00782, -0.0001982, 2.321e-06, -1.048e-08],
             [-2.25, 0.0984, -0.002803, 3.373e-05, -1.466e-07],
             [-5.924, 0.4878, -0.01567, 0.000213, -1.038e-06],
             [-1.635, 0.2794, -0.01421, 0.0002896, -1.959e-06],
             [0.7305, -0.06282, 0.001231, 1.65e-05, -5.488e-07]],
        'CTturb_J_fit_coeffs': [-0.2508, -0.2906, 0.6945],
        'cp_min': -5.0,
        'ct_max': 2.582,
        'ct_min': -0.228,
        'mach_max': 0.9,
        'mach_min': 0.3,
        'source': 'gen4_8p_tables_2019_08_15.json',
        'v_min': 30.0
    }
  }

  gen4rotor_mx_fit_coefficients = (
      gen4rotor_pitch_mx_fit_coefficients[fixed_rotor_pitch])

  cp_min = gen4rotor_mx_fit_coefficients['cp_min']
  ct_min = gen4rotor_mx_fit_coefficients['ct_min']
  ct_max = gen4rotor_mx_fit_coefficients['ct_max']
  mach_min = gen4rotor_mx_fit_coefficients['mach_min']
  mach_max = gen4rotor_mx_fit_coefficients['mach_max']
  v_min = gen4rotor_mx_fit_coefficients['v_min']

  def CTturb_J(x, x0, p, y0):
    """Function that provides a good fit to go from the turbine definition of
    thrust coefficient to the advance ratio."""
    # If ct is below ct_min, the stall margin constraint should get it back to
    # proper values, but saturate the value here to prevent log(<=0) warnings.
    x_lookup = np.maximum(x, x0 + 0.001)
    # Saturate the return to prevent nonsensical values from reaching
    # sqrts and denominators.
    return np.maximum(np.log((x_lookup-x0)) * p + y0, 0.001)

  def gen4_rotors_calc_shaft_power(rho, c_sound, v_ax, rotor_force):
    """Calculate rotor shaft power for a fixed-pitch rotor

    Args:
      rho: Air density [kg/m^3]
      c_sound: Speed of sound [m/s]
      v_ax: Airspeed along the rotor axis [m/s].
      rotor_force: Axial force for all rotors, positive is thrusting [N].

    Returns:
      status_update: dictionary containing:
        list of constraints dictionary: eg,
          c_t_margin: rotor stall margin, negative means stalled
        power_shaft: shaft power of all rotors, positive is generating [W]
        c_p
        c_t
        eta_rotors
        adv_ratio
        tip_mach
        omega_rotor
        torque_shaft
        power_shaft
    """

    # convert rotor_force into a thrust coefficient
    ct = rotor_force / (1.0 / 2.0 * rho * v_ax**2.0 * a_rotors)

    adv_ratio = CTturb_J(
        ct, *gen4rotor_mx_fit_coefficients['CTturb_J_fit_coeffs'])
    omega_rotor = np.pi * v_ax / (adv_ratio * r_rotor)
    tip_mach = np.sqrt(v_ax**2 + (omega_rotor * r_rotor)**2) / c_sound

    # Saturate the inputs to the fit lookup so that the output is sensible and
    # doesn't lead to wild extrapolation that might confuse the optimizer. The
    # fit constraint will be useful for understanding if/when we violate our
    # look-up domain and help push us back in, but the optimizer will likely
    # still perform better if things are well behaved in this region. The
    # saturation here focuses on the areas we are most likely to exceed and the
    # ones where the extrapolated fit diverges the most.

    # X-Rotor tables are fairly flat at low wind speeds and the fit can diverge.
    v_lookup = np.maximum(v_min, v_ax)
    # The efficiency should continue to die off at high mach numbers, (or at
    # least remain poor), but the extrapolation tends to diverge back up.
    # This saturates the max velocity used for the fit lookup to that at our
    # highest mach fit.
    v_lookup = np.minimum(
      mach_max * c_sound / np.sqrt(1. + (np.pi / adv_ratio)**2),
      v_lookup
    )
    # Beyond stall, the Cp should drop back down again, but this will at least
    # ensure it doesn't go up.
    ct_lookup = np.maximum(ct_min, ct)
    cp = np.polynomial.polynomial.polyval2d(
        ct_lookup, v_lookup,
        gen4rotor_mx_fit_coefficients['CTturb_CPturb_fit_coeffs'])

    power_shaft = cp * (1.0/2.0 * rho * v_ax**3.0 * a_rotors)
    stall_margin = ct - ct_min

    # Negative sign is because ct is positive when thrusting (adding energy
    # to the wind), but cp is positive when generating (taking energy out of
    # the wind).
    # Saturate min etas so they don't blow out the plot limits
    eta_rotors = np.minimum(np.maximum(-(cp/ct), -1.0),
                            np.maximum(-(ct/cp), -1.0))

    # Areas of the rotor were masked for cleaner fits in the operation zone.
    # Quantify how close we are to exceeding the fit domain.
    fit_margin = np.min([(ct - ct_min) / np.abs(ct_max-ct_min),
                     (ct_max - ct) / np.abs(ct_max-ct_min),
                     (mach_max - tip_mach) / 1.,
                     (tip_mach - mach_min) / 1.,
                     (v_ax - v_min) / v_min,
                     (cp - cp_min) / np.abs(cp_min)],
                     axis=0)

    torque_shaft = (power_shaft / n_rotors) / omega_rotor
    # The rotor_fit_margin is a check that we are not extrapolating beyond
    # the domain where we fit the polynomials. It's a modeling limitation, not
    # a physical limitation, so if this limit is being hit then we should
    # revisit the fits.
    # The stall margin checks the ct_min side of the domain.
    constraints = [{'name': 'rotor_stall_margin',
                    'value': ct,
                    'limit': ct_min,
                    'lim_type': 'min',
                    'margin': stall_margin,
                    'margin_norm': stall_margin / np.abs(ct_min)},
                   {'name': 'rotor_fit_margin',
                    'value': fit_margin,
                    'limit': 0,
                    'lim_type': 'min',
                    'margin': fit_margin,
                    'margin_norm': fit_margin / 1.},]
    if rotor_mach_limit is not None:
      rotor_mach_margin = rotor_mach_limit - tip_mach
      constraints += [{'name': 'rotor_mach_margin',
                      'value': tip_mach,
                      'limit': rotor_mach_limit,
                      'lim_type': 'max',
                      'margin': rotor_mach_margin,
                      'margin_norm': rotor_mach_margin / rotor_mach_limit}]

    state_update = {'c_p': cp,
                    'c_t': ct,
                    'eta_rotors': eta_rotors,
                    'power_shaft': power_shaft,
                    'aero_thrust_power': rotor_force * v_ax,
                    'thrust': rotor_force,
                    'constraints': constraints,
                    'adv_ratio': adv_ratio,
                    'tip_mach': tip_mach,
                    'omega_rotor': omega_rotor,
                    'torque_shaft': torque_shaft,
                    'constraints': constraints}

    return state_update

  return gen4_rotors_calc_shaft_power


def PlotRotor(shaft_power_from_drag_power, plot_spacing=False, **kwargs):
  """Plot the rotor table for a given rotor and config.

  args:
    shaft_power_from_drag_power: The rotor function for/from a kite config.
    plot_spacing: If true, also plots the grid of points from v_axs and
      rotor_forces so the user can judge if it has a reasonable density for
      generating the contour plots.
  main kwargs:
    v_axs, rotor_forces: Tuples that get passed to linspace to make the
      underlying grid. Note that rotor_forces is total force for all rotors,
      so if passing a rotor function for a single rotor setup, the values here
      should be set accordingly.
    power/torque_shaft_max: If passed, their constraint lines will be added
      to the plot. Note that the torque_shaft_max is defined per motor but the
      power_shaft_max limit is total for all rotors combined.
    x, y, zs: String (or list of strings for zs) of the variable name to plot.
      Must be variables that are in the state_update dictionary that the rotor
      function returns, or the 'rotor_force' or 'v_a_along_rotor_axis' that
      were passed into the rotor function.



  """

  rho = kwargs.get('rho', 1.225)
  c_sound = kwargs.get('c_sound', 343.0)
  v_axs = np.linspace(*kwargs.get('v_axs', (20, 90, 200)))
  rotor_forces = np.linspace(*kwargs.get(
      'rotor_forces', (-6000.*8, 4000.*8, 200)))
  power_shaft_max = kwargs.get('power_shaft_max', None)
  torque_shaft_max = kwargs.get('torque_shaft_max', None)
  x = kwargs.get('x', 'omega_rotor')
  y = kwargs.get('y', 'v_a_along_rotor_axis')
  zs = kwargs.get('zs', ['c_t', 'c_p', 'eta_rotors'])
  levels = kwargs.get('levels', 12)
  map_alpha = kwargs.get('map_alpha', 0.85)
  colormap = kwargs.get('colormap', 'summer')
  color = kwargs.get('color', 'C0')
  title = kwargs.get('title', '')
  legend_loc = kwargs.get('legend_loc', 'upper left')
  figsize = kwargs.get('figsize', (9,7))
  only_positive_eta = kwargs.get('only_positive_eta', True)
  show_contour_lines = kwargs.get('show_contour_lines', True)
  show_map = kwargs.get('show_map', True)
  extra_contours = kwargs.get('extra_contours', None)
  contour_label_fmt = kwargs.get('contour_label_fmt', '%.2f')
  skip_contours = kwargs.get('skip_contours', [])

  rotor_force_grid, v_ax_grid = np.meshgrid(rotor_forces, v_axs)
  table = shaft_power_from_drag_power(rho, c_sound, v_ax_grid, rotor_force_grid)
  table['rotor_force'] = rotor_force_grid
  table['v_a_along_rotor_axis'] = v_ax_grid

  fig, axes = plt.subplots(figsize=(figsize[0],figsize[1] * len(zs)),
                               nrows=len(zs), ncols=1, dpi=100)
  if len(zs) == 1:
    # Axes must be iterable to be later zipped with the zs.
    axes = [axes]

  # Convert list of constraints to dictionary.
  constraints = {
      constraint.pop('name'):constraint for constraint in table['constraints']}
  # Power and torque constraints are part of the kite config, not the rotor,
  # so if these limits were passed then add the constraint entries needed to
  # add to the plot.
  if power_shaft_max is not None:
    power_shaft_abs = np.abs(table['power_shaft'])
    constraints['power_shaft_max'] = {
        'limit': power_shaft_max,
        'value': power_shaft_abs,
        'margin_norm': (power_shaft_abs - power_shaft_max) / power_shaft_max}
  if torque_shaft_max is not None:
    torque_shaft_abs = np.abs(table['torque_shaft'])
    constraints['torque_shaft_max'] = {
        'limit': torque_shaft_max,
        'value': torque_shaft_abs,
        'margin_norm': (torque_shaft_abs - torque_shaft_max) / torque_shaft_max}
  constraint_colors = ['purple', 'blue', 'red', 'magenta', 'orange']
  # All the points of the look-up grid that are within the valid region of the
  # fit.
  fit_mask = constraints['rotor_fit_margin']['value'] >= 0.
  eta_rotor_mask = table['eta_rotors'] >= 0.

  for ax, z in zip(axes, zs):
    if show_map:
      # Filled countor of the valid fit region with high res levels.
      color_levels = 3*(levels if type(levels) is int else len(levels))
      if z == 'eta_rotors' and only_positive_eta:
        temp_mask = fit_mask * eta_rotor_mask
      else:
        temp_mask = fit_mask
      IM = ax.tricontourf(
        table[x][temp_mask].flatten(),
        table[y][temp_mask].flatten(),
        table[z][temp_mask].flatten(),
        levels*3, alpha=map_alpha,
        cmap=colormap)
    if show_contour_lines:
      # Use the range of only the fitted region for setting the levels, unless
      # an actual list of levels was passed.
      line_levels = (np.linspace(np.min(table[z][fit_mask].flatten()),
                        np.max(table[z][fit_mask].flatten()), levels+2)
                    if type(levels) is int else levels)
      # Add contour lines
      CS = ax.tricontour(
        table[x][fit_mask].flatten(),
        table[y][fit_mask].flatten(),
        table[z][fit_mask].flatten(),
        line_levels,
        colors='k',
        linewidths=1)
      ax.clabel(CS, inline=1, inline_spacing=0.5, fmt=contour_label_fmt)
    if extra_contours is not None:
      ECS = ax.tricontour(
          table[x][fit_mask].flatten(),
          table[y][fit_mask].flatten(),
          table[extra_contours[0]][fit_mask].flatten(),
          levels=extra_contours[1],
          colors=extra_contours[2],
          linewidths=1.)
      ax.clabel(ECS, inline=1, inline_spacing=0.5, fmt=contour_label_fmt)
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()

    # Make an explicit 0 contour line and make it more visible.
    CS0 = ax.tricontour(
      table[x][fit_mask].flatten(),
      table[y][fit_mask].flatten(),
      table[z][fit_mask].flatten(),
      [0.],
      colors='k',
      linewidths=3)
    ax.clabel(CS0, inline=1, inline_spacing=0.5, fmt=contour_label_fmt, fontsize='x-large')
    for ii, (k, v) in enumerate(constraints.items()):
      # Add contour line for each constraint.
      if k != 'rotor_fit_margin' and k not in 'skip_contours':
        ax.tricontour(
          table[x][fit_mask].flatten(),
          table[y][fit_mask].flatten(),
          v['value'][fit_mask].flatten(),
          levels=[v['limit']],
          colors=constraint_colors[ii],
          linewidths=3,
        )
        # Add constraint limits to legend.
        ax.plot(np.nan, np.nan, color=constraint_colors[ii], label=k)
    # Set the limits to slightly larger than the fitted region.
    lim_margin = 0.05
    ax.set_xlim((xlim[0] - lim_margin * (xlim[1] - xlim[0])),
                (xlim[1] + lim_margin * (xlim[1] - xlim[0])))
    ax.set_ylim((ylim[0] - lim_margin * (ylim[1] - ylim[0])),
                (ylim[1] + lim_margin * (ylim[1] - ylim[0])))
    ax.set_xlabel(x)
    ax.set_ylabel(y)
    ax.set_title(z)
    if show_map:
      fig.colorbar(IM, ax=ax)
    ax.legend(loc=legend_loc)
  fig.suptitle(title + '\nNotes: Colored region is where the fit is valid;' +
    '\n Coefficients are defined using wind turbine notation.')

  # Plot the points on the underlying grid that is going into the contour plots.
  # Just needed to inspect/debug if the contour plot is doing something odd.
  if plot_spacing:
    spacing_fig, spacing_ax = plt.subplots(1,1, figsize=figsize)
    spacing_ax.plot(
          table[x].flatten(),
          table[y].flatten(),
          '.', markersize=1)
    spacing_ax.set_xlim(xlim)
    spacing_ax.set_ylim(ylim)

  return fig
