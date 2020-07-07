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

"""Class for managing properties of an airfoil."""

import copy
import importlib
import json
import logging
import os

import makani
import numpy
import scipy.interpolate


class Airfoil(object):
  """Class for managing properties of an airfoil."""

  def __init__(self, filename, stall_angles_deg=None):
    """Loads airfoil data from a JSON file.

    Args:
      filename: Full path to JSON airfoil parameter file.
      stall_angles_deg: Low and high angle [deg] at which the airfoil stalls.
    """
    self._filename = filename
    with open(self._filename, 'r') as f:
      airfoil_data = json.load(f)

    # Only mix with post stall data if stall angles are specified.
    if stall_angles_deg:
      assert stall_angles_deg[1] > stall_angles_deg[0]
      self._stall_angles = [numpy.pi / 180.0 * x for x in stall_angles_deg]
      airfoil_data = self._AddPostStallData(airfoil_data, self._stall_angles)
    else:
      self._stall_angles = [-numpy.inf, numpy.inf]

    self._name = airfoil_data['name']
    self._alphas = numpy.array(airfoil_data['alphas'])
    self._alphas_deg = 180.0 / numpy.pi * self._alphas
    self._flap_deflections = numpy.array(airfoil_data['flap_deflections'])
    self._flap_deflections_deg = 180.0 / numpy.pi * self._flap_deflections
    self._min_flap_deflection = numpy.min(self._flap_deflections)
    self._max_flap_deflection = numpy.max(self._flap_deflections)

    self._lift_coeffs = numpy.array(airfoil_data['lift_coeffs'])
    self._drag_coeffs = numpy.array(airfoil_data['drag_coeffs'])
    self._moment_coeffs = numpy.array(airfoil_data['moment_coeffs'])

    # Create interpolation functions for each coefficient.
    self._lift_coeff = self._GetInterpolation(self._flap_deflections,
                                              self._alphas, self._lift_coeffs)
    self._drag_coeff = self._GetInterpolation(self._flap_deflections,
                                              self._alphas, self._drag_coeffs)
    self._moment_coeff = self._GetInterpolation(self._flap_deflections,
                                                self._alphas,
                                                self._moment_coeffs)

  def __str__(self):
    return '%s with stall angles (%0.3f, %0.3f) [deg].' % (
        os.path.basename(self._filename),
        (180.0 / numpy.pi) * self._stall_angles[0],
        (180.0 / numpy.pi) * self._stall_angles[1])

  def _GetInterpolation(self, flap_deflections, alphas, coeffs):
    if len(flap_deflections) > 1:
      return scipy.interpolate.RectBivariateSpline(
          flap_deflections, alphas, numpy.array(coeffs), kx=1, ky=1).ev
    else:
      return lambda _, x: scipy.interp(x, alphas, coeffs[0])

  def _AddPostStallData(self, airfoil_data, stall_angles):
    """Blends airfoil data with high angle-of-attack airfoil data.

    Creates an airfoil data structure that uses the normal airfoil
    data within the stall angles and blends to the high
    angle-of-attack data outside these angles.

    Args:
      airfoil_data: Dictionary containing the airfoil force and
          moment coefficients [#] as a function of angle-of-attack [rad].
      stall_angles: Angles [rad] outside which to blend to the high
          angle-of-attack airfoil data.

    Returns:
      Dictionary of airfoil force and moment coefficients.
    """

    # Perform sanity checks on inputs.
    if min(airfoil_data['alphas']) > stall_angles[0]:
      logging.warning('Lower stall angle is beyond range of airfoil data.')
    if max(airfoil_data['alphas']) < stall_angles[1]:
      logging.warning('Upper stall angle is beyond range of airfoil data.')
    assert (numpy.diff(airfoil_data['alphas']) > 0.0).all()

    with open(os.path.join(makani.HOME,
                           'analysis/aero/airfoils/naca0012_high_aoa.json'),
              'r') as f:
      naca0012_high_aoa = json.load(f)

    alphas = numpy.linspace(-numpy.pi, numpy.pi, 400)
    full_airfoil_data = copy.deepcopy(airfoil_data)
    full_airfoil_data['alphas'] = alphas.tolist()

    angles_past_stall = numpy.maximum(
        numpy.maximum(stall_angles[0] - alphas, alphas - stall_angles[1]),
        0.0)
    weights = 1.0 - numpy.minimum(angles_past_stall / 0.1, 1.0)

    for coeff in ('lift_coeffs', 'drag_coeffs', 'moment_coeffs'):
      full_airfoil_data[coeff] = numpy.zeros(
          (len(airfoil_data['flap_deflections']), len(alphas)))
      low_aoa = self._GetInterpolation(airfoil_data['flap_deflections'],
                                       airfoil_data['alphas'],
                                       airfoil_data[coeff])
      high_aoa = self._GetInterpolation(naca0012_high_aoa['flap_deflections'],
                                        naca0012_high_aoa['alphas'],
                                        naca0012_high_aoa[coeff])
      for i, flap_deflection in enumerate(airfoil_data['flap_deflections']):
        for j in range(len(alphas)):
          full_airfoil_data[coeff][i][j] = (
              weights[j] * low_aoa(flap_deflection, alphas[j]) +
              (1.0 - weights[j]) * high_aoa(flap_deflection, alphas[j]))

    return full_airfoil_data

  def Plot(self):
    """Plots airfoil coefficients as a function of angle-of-attack."""

    # b/120081442: Next line removed the module initialization load of the
    # matplotlib module which was causing a bazel pip-installed package issue on
    # batch sim workers.
    pyplot = importlib.import_module('matplotlib.pyplot')
    fig = pyplot.figure()
    axes = fig.add_subplot(1, 1, 1)
    for i, flap_deflection_deg in enumerate(self._flap_deflections_deg):
      pyplot.plot(self._alphas_deg, self._lift_coeffs[i], '.',
                  label='CL (%d deg)' % flap_deflection_deg)
      pyplot.plot(self._alphas_deg, self._drag_coeffs[i], '.',
                  label='CD (%d deg)' % flap_deflection_deg)
      pyplot.plot(self._alphas_deg, self._moment_coeffs[i], '.',
                  label='Cm (%d deg)' % flap_deflection_deg)

    axes.set_title(self._name)
    axes.set_xlabel('Angle-of-attack [deg]')
    axes.set_ylabel('Coefficient [#]')
    axes.grid(True)
    pyplot.legend(loc='lower left', prop={'size': 10})

    pyplot.gca().set_color_cycle(None)
    for i, flap_deflection in enumerate(self._flap_deflections):
      pyplot.plot(self._alphas_deg,
                  self._lift_coeff(flap_deflection, self._alphas).T, '-',
                  label='CL (%d deg)' % self._flap_deflections_deg[i])
      pyplot.plot(self._alphas_deg,
                  self._drag_coeff(flap_deflection, self._alphas).T, '-',
                  label='CD (%d deg)' % self._flap_deflections_deg[i])
      pyplot.plot(self._alphas_deg,
                  self._moment_coeff(flap_deflection, self._alphas).T, '-',
                  label='Cm (%d deg)' % self._flap_deflections_deg[i])

    pyplot.show(block=False)

  def GetCoeffs(self, angle, delta=0.0):
    """Calculates airfoil coefficients.

    Calculates airfoil coefficients at a specified incidence angle and
    flap deflection.

    Args:
      angle: Incidence angle [rad].
      delta: Flap deflection angle [rad].

    Returns:
      Lift, drag, and moment coefficients at the specified incidence
      angle and flap deflection.
    """

    wrapped_angle = ((angle + numpy.pi) % (2.0 * numpy.pi)) - numpy.pi
    saturated_delta = numpy.minimum(numpy.maximum(
        delta, self._min_flap_deflection), self._max_flap_deflection)

    # Get everything into the same shape.
    if len(numpy.shape(angle)) > 1:
      shape = numpy.shape(angle)
      wrapped_angle = wrapped_angle.flatten()
      if len(numpy.shape(delta)) < 1:
        saturated_delta *= numpy.ones(shape)
      saturated_delta = saturated_delta.flatten()

    # Perform lookup.
    lift_coeff = self._lift_coeff(saturated_delta, wrapped_angle)
    drag_coeff = self._drag_coeff(saturated_delta, wrapped_angle)
    moment_coeff = self._moment_coeff(saturated_delta, wrapped_angle)

    # Reshape everything as we found it.
    if len(numpy.shape(angle)) > 1 or len(numpy.shape(delta)) > 1:
      lift_coeff = numpy.reshape(lift_coeff, shape)
      drag_coeff = numpy.reshape(drag_coeff, shape)
      moment_coeff = numpy.reshape(moment_coeff, shape)

    return lift_coeff, drag_coeff, moment_coeff
