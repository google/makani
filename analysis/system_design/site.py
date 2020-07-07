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

"""Module for wind site modeling within system design tool.

This module is intended to model the properties associated with a wind site that
impact the determination of the levelized cost of energy at the wind farm level.
In its current state there are no functions, so it is just a container for the
associated characteristics.
"""


class Site(object):
  """Models a wind site.

  This class contains characteristics that are specific to an individual wind
  site. This includes the probability distribution of specific wind velocities,
  the probability distribution of the wind shear coefficient, and the initial
  capital costs.

  Attributes:
    site_name: A string containing the name of the site.
    velocity_distribution: A dictionary containing velocity and probability
        information specific to the site.
    shear_distribution: A dictonary containing shear coefficient and probability
        information specific to the site.
    capital_costs: A float describing the initial capital cost for the site.
  """

  def __init__(self, site_name=None, velocity_distribution=None,
               shear_distribution=None, capital_costs=None):
    """Constructor.

    Args:
      site_name: (Optional) String containing name of site.
      velocity_distribution: (Optional) Dictionary with keys 'velocity' and
          'probability' that have values of 1D lists of equal length containing
          sets of velocity and the associated probability of that velocity.
          Velocity values must be floats monotonically increasing from 0.0, and
          probability values must be floats that are zero or greater and sum to
          1.0.
      shear_distribution: (Optional) Dictionary with keys 'shear_coefficient'
          and 'probability' that have 1D lists of equal length containing sets
          of shear coefficient and the associated probability of that shear
          coefficient. Shear coefficient values must be floats monotonically
          increasing from 0.0, and probability values must be floats that are
          zero or greater and sum to 1.0.
      capital_costs: (Optional) Initial capital cost [$USD/m^2] for the site.
    """

    # TODO: Allow for wind class as a method of definition.
    # TODO: Add description wind shear with reference height.
    # TODO: Add input checking.

    self._site_name = 'Default Site Name' if site_name is None else site_name

    if velocity_distribution is None:
      self._velocity_distribution = {
          'velocity': [0.0, 10.0, 20.0],
          'probability': [0.2, 0.6, 0.2]
      }
    else:
      self._velocity_distribution = velocity_distribution

    if shear_distribution is None:
      self._shear_distribution = {
          'shear_coefficient': [0.0, 0.1, 0.2],
          'probability': [0.0, 0.9, 0.1]
      }
    else:
      self._shear_distribution = shear_distribution

    self._capital_costs = 1.0 if capital_costs is None else capital_costs

  @property
  def site_name(self):
    return self._site_name

  @property
  def velocity_distribution(self):
    return self._velocity_distribution

  @property
  def shear_distribution(self):
    return self._shear_distribution

  @property
  def capital_costs(self):
    return self._capital_costs
