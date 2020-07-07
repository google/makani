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

"""Crosswind playbook parameters."""

from makani.control import control_types as m


def GetPlaybook(test_site):
  """Creates crosswind playbook.

  Crosswind playbooks are outer loop control commands that change with
  wind speed.

  Args:
    test_site: Integer specifying test site, as defined in TestSite enum.

  Returns:
    Dictionary containing crosswind playbook and associated params.
  """
  if test_site == m.kTestSiteParkerRanch:

    # OktoberKite playbook is from FBL results from power curve results
    # for BigM600_r07c_v01-circle_flap2_5.

    # Wind adjust params rescale the wind speeds used in the playbook lookup.
    # Low and high winds are generally riskier than moderate winds, so these
    # parameters can enable compression of the playbook (via scaling_factor
    # < 1) around a central "crossover" point so the kite switches to safer
    # entries sooner.
    # The intent is that at high winds we fly a higher wind speed solution
    # than optimized for by some margin, and at low winds we fly a lower
    # wind speed solution. Since these solutions are considered to be for
    # extreme wind cases, we are reaching the extreme solution at a slightly
    # less extreme wind with the hopes that our behavior will be more
    # conservative.
    # A scaling_factor of 1 will have no effect.

    # Wind adjust params for Oktoberkite currently set to have no effect.
    wind_adjust_params = {'crossover': 9.0,
                          'min_offset': 0.,
                          'scaling_factor': 1.}

    transout_airspeed_smooth_weights = [
        [0.01, 0.1, 0.01],
        [0.05, 0.2, 0.05],
        [0.10, 0.3, 0.10],
        [0.05, 0.2, 0.05],
        [0.01, 0.1, 0.01]]

    # Playbooks from FBL do not require additional smoothing.
    # All smoothing weights are singular.
    smoothing_params = {
        'alpha_lookup': {'weights': [[1.]],
                         'edges': ['extend', 'continuous']},
        'beta_lookup': {'weights': [[1.]],
                        'edges': ['extend', 'continuous']},
        'airspeed_lookup': {'weights': [[1.]],
                            'edges': ['extend', 'continuous']},
        'transout_airspeed_lookup': {
            'weights': transout_airspeed_smooth_weights,
            'edges': ['extend', 'continuous']},
        'elevation': {'weights': [1.],
                      'edges': ['extend']},
        'azi_offset': {'weights': [1.],
                       'edges': ['extend']},
        'path_radius_target': {'weights': [1.],
                               'edges': ['extend']}}

    # Single values are a simple lookup, while multiple values represent
    # values for cubic spline control points, evenly spaced with loop angle.
    playbook_parameterized = [
        {'airspeed': [31.6443, 40.6491, 47.0736, 43.9876, 32.1313, 30.7405],
         'alpha': [-0.0812, -0.02, -0.0027, -0.0009, -0.0131, -0.0846],
         'azi_offset': 0.0546,
         'beta': [-0.0384, -0.0384, -0.0384, -0.0384, -0.0384, -0.0384],
         'elevation': 0.5098,
         'radius': 95.91,
         'wind_speed': 4.0},
        {'airspeed': [32.8133, 44.446, 51.7089, 50.5689, 39.9426, 31.3679],
         'alpha': [-0.0393, -0.0055, -0.0004, -0.0004, -0.0085, -0.0507],
         'azi_offset': 0.0253,
         'beta': [-0.0482, -0.0482, -0.0482, -0.0482, -0.0482, -0.0482],
         'elevation': 0.4891,
         'radius': 90.0017,
         'wind_speed': 6.0},
        {'airspeed': [37.2616, 50.3141, 57.3948, 54.225, 46.1069, 34.9328],
         'alpha': [-0.0026, -0.0002, 0.0, -0.0002, -0.0026, -0.0138],
         'azi_offset': 0.0081,
         'beta': [-0.0474, -0.0474, -0.0474, -0.0474, -0.0474, -0.0474],
         'elevation': 0.4891,
         'radius': 90.0003,
         'wind_speed': 8.0},
        {'airspeed': [49.5387, 58.2034, 63.8272, 60.2092, 52.3286, 47.11],
         'alpha': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
         'azi_offset': 0.0045,
         'beta': [-0.0521, -0.0521, -0.0521, -0.0521, -0.0521, -0.0521],
         'elevation': 0.4891,
         'radius': 90.0,
         'wind_speed': 10.0},
        {'airspeed': [50.9564, 59.2743, 67.3115, 65.8786, 57.8558, 52.2734],
         'alpha': [-0.0034, -0.0029, -0.0316, -0.0174, -0.0008, -0.0021],
         'azi_offset': 0.0765,
         'beta': [-0.0522, -0.0522, -0.0522, -0.0522, -0.0522, -0.0522],
         'elevation': 0.4891,
         'radius': 90.0,
         'wind_speed': 12.0},
        {'airspeed': [53.1502, 62.4041, 71.581, 70.8717, 63.42, 55.079],
         'alpha': [-0.001, -0.0253, -0.0545, -0.0529, -0.0065, -0.0015],
         'azi_offset': 0.155,
         'beta': [-0.053, -0.053, -0.053, -0.053, -0.053, -0.053],
         'elevation': 0.4891,
         'radius': 90.0,
         'wind_speed': 14.0},
        {'airspeed': [55.7261, 62.5564, 71.9207, 72.8428, 67.5872, 60.2103],
         'alpha': [-0.0218, -0.0351, -0.0561, -0.0657, -0.0293, 0.0012],
         'azi_offset': 0.3165,
         'beta': [-0.0475, -0.0475, -0.0475, -0.0475, -0.0475, -0.0475],
         'elevation': 0.548,
         'radius': 91.7197,
         'wind_speed': 16.0},
        {'airspeed': [58.2542, 64.2707, 73.2495, 74.7305, 70.5466, 63.7057],
         'alpha': [-0.0069, -0.0304, -0.0612, -0.0758, -0.0485, -0.0038],
         'azi_offset': 0.3717,
         'beta': [-0.0584, -0.0584, -0.0584, -0.0584, -0.0584, -0.0584],
         'elevation': 0.6304,
         'radius': 91.0864,
         'wind_speed': 18.0},
        {'airspeed': [57.3239, 61.8278, 72.2796, 75.9291, 72.9086, 65.5985],
         'alpha': [0.0001, -0.0168, -0.0563, -0.0842, -0.0648, -0.0162],
         'azi_offset': 0.4307,
         'beta': [-0.0348, -0.0348, -0.0348, -0.0348, -0.0348, -0.0348],
         'elevation': 0.7067,
         'radius': 96.7637,
         'wind_speed': 20.0}]
  else:
    # There is no offshore playbook for OktoberKite.
    assert False, (
        'Oktoberkite must be flown onshore at Parker Ranch. '
        'No offshore playbook.')

  # Transout airspeed commands are modified versions of the V1 Playbook airspeed
  # schedules. The original schedules were adjusted to make the commands both
  # slower and flatter at the 6 o'clock loop position to help the kite slow
  # down without throwing the bridle forward such that it contacts pylons.
  #
  # As this playbook was shorter, it's extended for higher wind
  # speeds. These airspeeds are scheduled with wind speed from 3 to 19 m/s
  # at a 2 m/s interval.
  # TODO(b/137710129): Make aligning of these airspeed schedules to the playbook
  # wind speeds more robust.
  # TODO: Trans-out airspeeds need a revision for the OktoKite.
  transout_airspeeds = [[35.0, 40.0, 43.5, 43.5, 41.0, 40.0],
                        [35.0, 40.8, 44.1, 44.1, 42.0, 39.5],
                        [35.0, 41.5, 44.8, 44.8, 43.0, 39.0],
                        [37.5, 42.2, 45.4, 45.4, 44.0, 38.5],
                        [40.0, 43.0, 46.0, 46.0, 45.0, 38.0],
                        [42.0, 45.0, 48.0, 48.0, 47.0, 40.0],
                        [44.0, 47.0, 50.0, 50.0, 49.0, 42.0],
                        [46.0, 49.0, 52.0, 52.0, 51.0, 44.0],
                        [48.0, 51.0, 54.0, 54.0, 53.0, 46.0]]

  return {
      'wind_adjust_params': wind_adjust_params,
      'smoothing_params': smoothing_params,
      'playbook_parameterized': playbook_parameterized,
      'transout_airspeeds': transout_airspeeds,
  }
