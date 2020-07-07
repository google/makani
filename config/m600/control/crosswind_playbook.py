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
  if test_site == m.kTestSiteNorway:

    # ECR465: Playbook version vF3 (P22os_r_s4) is a
    # smoothed version of a Vizier prototype 22 offshore smoothest playbook,
    # filtered for acceptable commands (alphas < 3 deg, etc),
    # with key wind speeds and repeatable performance emphasized,
    # azi-offset manually increased for both higher wind speeds and globally,
    # betas reduced, and more smoothness within a loop for alpha and airspeed.
    # Goal is safe, conservative operation across many wind speeds, without
    # regard for power.

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
    wind_adjust_params = {'crossover': 9.0,
                          'min_offset': -1.0,
                          'scaling_factor': 0.95}

    transout_airspeed_smooth_weights = [
        [0.01, 0.1, 0.01],
        [0.05, 0.2, 0.05],
        [0.10, 0.3, 0.10],
        [0.05, 0.2, 0.05],
        [0.01, 0.1, 0.01]]

    two_dim_smooth_weights = [
        [0., 0., 0.0009, 0.0042, 0.007, 0.0094, 0.0111, 0.0118, 0.0111,
         0.0094, 0.007, 0.0042, 0.0009, 0., 0.],
        [0., 0., 0.0036, 0.0071, 0.0103, 0.013, 0.015, 0.0158, 0.015, 0.013,
         0.0103, 0.0071, 0.0036, 0., 0.],
        [0., 0.0029, 0.007, 0.0109, 0.0145, 0.0178, 0.0202, 0.0213, 0.0202,
         0.0178, 0.0145, 0.0109, 0.007, 0.0029, 0.],
        [0.0028, 0.0075, 0.0123, 0.0171, 0.0221, 0.0272, 0.0326, 0.0389,
         0.0326, 0.0272, 0.0221, 0.0171, 0.0123, 0.0075, 0.0028],
        [0., 0.0029, 0.007, 0.0109, 0.0145, 0.0178, 0.0202, 0.0213, 0.0202,
         0.0178, 0.0145, 0.0109, 0.007, 0.0029, 0.],
        [0., 0., 0.0036, 0.0071, 0.0103, 0.013, 0.015, 0.0158, 0.015, 0.013,
         0.0103, 0.0071, 0.0036, 0., 0.],
        [0., 0., 0.0009, 0.0042, 0.007, 0.0094, 0.0111, 0.0118, 0.0111,
         0.0094, 0.007, 0.0042, 0.0009, 0., 0.]
    ]

    # Smoothing convolution kernel for variables that vary only with wind speed.
    one_dim_smooth_weights = [
        0., 0.0833, 0.1429, 0.1786, 0.1905, 0.1786, 0.1429, 0.0833, 0.]

    smoothing_params = {
        'alpha_lookup': {'weights': two_dim_smooth_weights,
                         'edges': ['extend', 'continuous']},
        'beta_lookup': {'weights': [[w] for w in one_dim_smooth_weights],
                        'edges': ['extend', 'continuous']},
        'airspeed_lookup': {'weights': two_dim_smooth_weights,
                            'edges': ['extend', 'continuous']},
        'transout_airspeed_lookup': {
            'weights': transout_airspeed_smooth_weights,
            'edges': ['extend', 'continuous']},
        'elevation': {'weights': one_dim_smooth_weights,
                      'edges': ['extend']},
        'azi_offset': {'weights': one_dim_smooth_weights,
                       'edges': ['extend']},
        'path_radius_target': {'weights': one_dim_smooth_weights,
                               'edges': ['extend']}}

    # Single values are a simple lookup, while multiple values represent
    # values for cubic spline control points, evenly spaced with loop angle.
    playbook_parameterized = [{
        'airspeed': [42.8881, 51.0821, 56.0029, 53.5837, 41.0664, 45.0528],
        'alpha': [-0.0116, -0.0029, -0.0328, 0.0143, 0.0129, -0.0108],
        'azi_offset': 0.0536,
        'beta': [0.0338, 0.0338, 0.0338, 0.0338, 0.0338, 0.0338],
        'elevation': 0.7492,
        'radius': 168.7215,
        'wind_speed': 3.0
    }, {
        'airspeed': [39.5219, 41.3904, 51.9468, 43.9145, 45.1777, 38.3911],
        'alpha': [0.0287, -0.0216, 0.017, 0.0439, 0.0371, 0.0002],
        'azi_offset': 0.0975,
        'beta': [0.0381, 0.0381, 0.0381, 0.0381, 0.0381, 0.0381],
        'elevation': 0.6551,
        'radius': 143.5465,
        'wind_speed': 5.0
    }, {
        'airspeed': [41.468, 45.871, 57.7906, 48.1485, 46.0789, 45.2086],
        'alpha': [-0.012, 0.0138, 0.052, 0.0485, 0.0329, 0.0338],
        'azi_offset': 0.2241,
        'beta': [0.0312, 0.0312, 0.0312, 0.0312, 0.0312, 0.0312],
        'elevation': 0.7033,
        'radius': 172.1219,
        'wind_speed': 7.0
    }, {
        'airspeed': [41.4965, 47.7177, 54.3855, 53.2432, 43.0036, 43.146],
        'alpha': [0.0212, 0.0126, 0.0523, 0.0339, 0.0231, 0.0348],
        'azi_offset': 0.2233,
        'beta': [0.0312, 0.0312, 0.0312, 0.0312, 0.0312, 0.0312],
        'elevation': 0.5822,
        'radius': 154.6331,
        'wind_speed': 9.0
    }, {
        'airspeed': [46.668, 58.118, 60.9573, 58.2572, 43.6144, 44.7395],
        'alpha': [0.0153, 0.0103, 0.0323, 0.0359, 0.0276, 0.0477],
        'azi_offset': 0.167,
        'beta': [0.0047, 0.0047, 0.0047, 0.0047, 0.0047, 0.0047],
        'elevation': 0.5727,
        'radius': 168.9258,
        'wind_speed': 11.0
    }, {
        'airspeed': [47.0302, 55.6071, 59.5185, 60.4036, 53.0888, 45.7893],
        'alpha': [0.0293, 0.0047, 0.0496, 0.0492, 0.0471, 0.0337],
        'azi_offset': 0.1663,
        'beta': [-0.0143, -0.0143, -0.0143, -0.0143, -0.0143, -0.0143],
        'elevation': 0.5349,
        'radius': 168.1162,
        'wind_speed': 13.0
    }, {
        'airspeed': [61.3872, 56.0591, 66.91, 60.2903, 58.1559, 55.4706],
        'alpha': [0.0428, -0.0199, -0.0069, 0.0438, -0.0229, 0.0313],
        'azi_offset': 0.4712,
        'beta': [0.0397, 0.0397, 0.0397, 0.0397, 0.0397, 0.0397],
        'elevation': 0.7155,
        'radius': 159.6261,
        'wind_speed': 15.0
    }, {
        'airspeed': [44.4929, 54.9966, 64.4057, 64.859, 52.7095, 53.3922],
        'alpha': [-0.0034, 0.0401, 0.0372, 0.0397, 0.035, 0.0268],
        'azi_offset': 0.733,
        'beta': [-0.0494, -0.0494, -0.0494, -0.0494, -0.0494, -0.0494],
        'elevation': 0.8909,
        'radius': 156.725,
        'wind_speed': 17.0
    }]

  else:
    wind_adjust_params = {
        'min_offset': -1.0,
        'crossover': 9.0,
        'scaling_factor': 0.9
    }

    transout_airspeed_smooth_weights = [
        [0.01, 0.1, 0.01],
        [0.05, 0.2, 0.05],
        [0.10, 0.3, 0.10],
        [0.05, 0.2, 0.05],
        [0.01, 0.1, 0.01]]

    smoothing_params = {
        'alpha_lookup': {
            'edges': ['extend', 'continuous'],
            'weights': [
                [0.0, 0.0, 0.0, 0.0, 0.0084, 0.0506, 0.0084, 0.0, 0.0, 0.0,
                 0.0],
                [0.0, 0.0, 0.0, 0.0084, 0.0289, 0.0781, 0.0289, 0.0084, 0.0,
                 0.0, 0.0],
                [0.0043, 0.0168, 0.0317, 0.0506, 0.0781, 0.1968, 0.0781, 0.0506,
                 0.0317, 0.0168, 0.0043],
                [0.0, 0.0, 0.0, 0.0084, 0.0289, 0.0781, 0.0289, 0.0084, 0.0,
                 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0084, 0.0506, 0.0084, 0.0, 0.0, 0.0,
                 0.0]]},
        'beta_lookup': {'weights': [[1.]*3, [1.]*3, [1.]*3, [1.]*3, [1.]*3],
                        'edges': ['extend', 'continuous']},
        'airspeed_lookup': {
            'edges': ['extend', 'continuous'],
            'weights': [
                [0.0026, 0.0095, 0.0163, 0.0232, 0.0299, 0.0352, 0.0299, 0.0232,
                 0.0163, 0.0095, 0.0026],
                [0.0027, 0.0095, 0.0164, 0.0233, 0.0302, 0.0368, 0.0302, 0.0233,
                 0.0164, 0.0095, 0.0027],
                [0.0027, 0.0095, 0.0164, 0.0233, 0.0302, 0.0371, 0.0302, 0.0233,
                 0.0164, 0.0095, 0.0027],
                [0.0027, 0.0095, 0.0164, 0.0233, 0.0302, 0.0368, 0.0302, 0.0233,
                 0.0164, 0.0095, 0.0027],
                [0.0026, 0.0095, 0.0163, 0.0232, 0.0299, 0.0352, 0.0299, 0.0232,
                 0.0163, 0.0095, 0.0026]]},
        'transout_airspeed_lookup': {
            'weights': transout_airspeed_smooth_weights,
            'edges': ['extend', 'continuous']},
        'elevation': {
            'weights': [0.0, 0.1111, 0.2222, 0.3333, 0.2222, 0.1111, 0.0],
            'edges': ['extend']},
        'azi_offset': {'weights': [0.5, 1., 0.5],
                       'edges': ['extend']},
        'path_radius_target': {'weights': [0.5, 1., 0.5],
                               'edges': ['extend']}}

    # This is autogenerated from a Colab notebook.
    # Single values are a simple lookup, while multiple values represent
    # values for cubic spline control points, evenly spaced with loop angle.
    playbook_parameterized = ([
        {'airspeed': [41.0, 44.5, 52.5, 50.0, 41.5, 41.0],
         'alpha': [0.01745, 0.02451, 0.05339, 0.06467, 0.05346, 0.01858],
         'azi_offset': 0.1454,
         'beta': [0.0222, 0.0222, 0.0222, 0.0222, 0.0222, 0.0222],
         'elevation': 0.69,
         'radius': 175.0,
         'wind_speed': 3.0},
        {'airspeed': [41.0, 44.86517, 54.0, 44.0, 41.5, 41.0],
         'alpha': [0.01745, 0.01745, 0.06298, 0.06981, 0.06298, 0.01745],
         'azi_offset': 0.137,
         'beta': [0.0188, 0.0188, 0.0188, 0.0188, 0.0188, 0.0188],
         'elevation': 0.67,
         'radius': 160.0,
         'wind_speed': 5.0},
        {'airspeed': [41.0, 46.6842, 54.4602, 48.8415, 42.0, 41.0],
         'alpha': [0.01745, 0.0478, 0.08041, 0.08651, 0.0848, 0.06519],
         'azi_offset': 0.0982,
         'beta': [0.00305, 0.00305, 0.00305, 0.00305, 0.00305, 0.00305],
         'elevation': 0.6,
         'radius': 155.0,
         'wind_speed': 7.0},
        {'airspeed': [41.0, 50.023, 60.1149, 57.129, 42.0, 41.0],
         'alpha': [0.03, 0.0603, 0.08478, 0.08703, 0.08468, 0.06387],
         'azi_offset': 0.05,
         'beta': [-0.0384, -0.0384, -0.0384, -0.0384, -0.0384, -0.0384],
         'elevation': 0.55,
         'radius': 130.0,
         'wind_speed': 9.0},
        {'airspeed': [41.0, 50.0, 55.0, 53.7272, 43.40516, 41.0],
         'alpha': [0.035, 0.048, 0.08322, 0.08648, 0.081, 0.06],
         'azi_offset': 0.0,
         'beta': [-0.0384, -0.0384, -0.0384, -0.0384, -0.0384, -0.0384],
         'elevation': 0.4821,
         'radius': 125.0,
         'wind_speed': 11.0},
        {'airspeed': [41.0, 50.0, 60.0, 61.0, 50.0, 41.0],
         'alpha': [0.02, 0.075, 0.08322, 0.08322, 0.08322, 0.02876],
         'azi_offset': 0.06842,
         'beta': [-0.0384, -0.0384, -0.0384, -0.0384, -0.0384, -0.0384],
         'elevation': 0.5188,
         'radius': 140.0,
         'wind_speed': 13.0},
        {'airspeed': [41.0, 50.0, 60.0, 63.0, 55.0, 42.38935],
         'alpha': [0.01, 0.065, 0.07651, 0.07894, 0.03936, 0.035],
         'azi_offset': 0.32366,
         'beta': [-0.0384, -0.0384, -0.0384, -0.0384, -0.0384, -0.0384],
         'elevation': 0.57,
         'radius': 155.0,
         'wind_speed': 15.0},
        {'airspeed': [42.0, 47.32911, 60.0, 59.0, 55.0, 42.42028],
         'alpha': [0.00396, 0.02594, 0.03681, 0.04158, 0.06514, 0.01785],
         'azi_offset': 0.40113,
         'beta': [-0.0384, -0.0384, -0.0384, -0.0384, -0.0384, -0.0384],
         'elevation': 0.67,
         'radius': 160.0,
         'wind_speed': 17.0},
        {'airspeed': [43.0, 47.5, 55.0, 56.0, 53.0, 46.0],
         'alpha': [0.03, 0.06, 0.06, 0.06, 0.02315, 0.01479],
         'azi_offset': 0.40113,
         'beta': [-0.0384, -0.0384, -0.0384, -0.0384, -0.0384, -0.0384],
         'elevation': 0.9,
         'radius': 170.0,
         'wind_speed': 19.0}])

  # The following parameters are identical for onshore and offshore playbooks.

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
