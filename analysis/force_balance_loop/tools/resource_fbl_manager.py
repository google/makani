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

"""Manages resources for FBL Model.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
from scipy.special import gammaln

def _RPX03_v_w_at_height(height, v_w_at_ref):
  h_ref = 180.
  if height < h_ref:
      v_w = v_w_at_ref
  else:
      v_w = v_w_at_ref * (height / h_ref)**(-0.4)
  return v_w

def _parker_ranch_v_w_at_height(height, v_w_at_ref):
  # Source is linear fit model for coefficients determined from mquick data analysis output
  # https://docs.google.com/spreadsheets/d/15boO1rGtVMWL0hrNWhAcFStaX16AJDsTCykQhrFurPo/edit

  h_ref = 100.
  C3 = -0.0000000063513655 * v_w_at_ref + 0.0000001216696031
  C2 = 0.000005289125078 * v_w_at_ref - 0.0001158557377
  C1 = -0.0009377939697 * v_w_at_ref + 0.02758663833
  C0 = 1.047239512 * v_w_at_ref - 1.721776059

  v_w = C3*height**3 + C2 * height**2 + C1 * height + C0

  return v_w

def MakeResourceByShearAndHref(
    shear, h_ref, rho=1.225, c_sound=343.0, v_w_avg_h_ref=7.5):
  
  def v_w_at_height(height, v_w_at_ref):
    v_w = v_w_at_ref * (height / h_ref)**shear
    return v_w

  def get_v_w_prob_dist(self, v_w_at_h_ref_range=None, k_wiebull=2.):
    '''Returns a probability distribution list for v_w at 
    ref height that is the same length as v_w_range.'''
    #TODO: update with exact CDF function 
    
    if v_w_at_h_ref_range is None:
      v_w_at_h_ref_range = np.arange(0., 35., 0.5)
    else:
      v_w_at_h_ref_range = np.asarray(v_w_at_h_ref_range)
    
    v_w_avg_h_ref_alpha = (v_w_avg_h_ref / np.exp(gammaln(1 + 1.0/k_wiebull)))
    
    # generate distribution at h_ref
    cdf_v_w_at_h_ref = fun.weib_cdf(
      v_w_at_h_ref_range, v_w_avg_h_ref_alpha, self.k_wiebull)
    pdf = cdf_v_w_at_h_ref[1:] - cdf_v_w_at_h_ref[:-1]
    prob_v_w_at_h_ref = np.insert(pdf, 0, 0.)
    
    v_w_prob_dist = {'v_ws_at_h_ref': v_w_at_h_ref_range.tolist(),
                     'prob_v_w_at_h_ref': prob_v_w_at_h_ref.tolist(),
                     'cdf_v_w_at_h_ref': cdf_v_w_at_h_ref.tolist()}
    
    return v_w_prob_dist

  resource = {'rho': rho,
              'c_sound': c_sound,
              'v_w_at_height': v_w_at_height,
              'v_w_avg_h_ref': v_w_avg_h_ref,
              'shear': shear,
              'get_v_w_prob_dist': get_v_w_prob_dist}
  return resource


# Air density worst cases can be found here:
# https://docs.google.com/document/d/1X9IDfBcwhFZMTxqG6Ara84YPnKJkMP37l0TQ7xXQKTY/edit
# Speed of sound in air is 343 m/s for 20C.
# TODO: Update c_sound for test day configs;
# TODO: Parker Ranch day and 24 hrs are based on old, flawed analysis
#   for average wind speeds (time offset error). Revise avg wind speeds.
# maybe make look-up table for rho and c_sound as function of air temp,etc,
# or just use a worst-case number?

resources = {'CL_nom':{'rho': 1.075,
                       'c_sound': 343.0,
                       'v_w_at_height': lambda h, v_w: v_w,
                       'v_w_avg_h_ref': 8.0},
             'mx_nom': MakeResourceByShearAndHref(
                 0.2, 80., 1.17, 7.5),
             'mx_nom_offshore': MakeResourceByShearAndHref(
                 0.11, 80., 1.225, 10.),
             'mx_nom_no_shear': MakeResourceByShearAndHref(
                 0., 80., 1.17, 7.5),
             'RPX03':{'rho': 1.075,
                      'c_sound': 343.0,
                      'v_w_at_height': _RPX03_v_w_at_height,
                      'v_w_avg_h_ref': 7.97},
             'parker_day': {'rho': 1.075,
                            'c_sound': 343.0,
                            'v_w_at_height': _parker_ranch_v_w_at_height,
                            'v_w_avg_h_ref': 5.42},
             'parker_24': {'rho': 1.075,
                           'c_sound': 343.0,
                           'v_w_at_height': _parker_ranch_v_w_at_height,
                           'v_w_avg_h_ref': 6.75}}


def GetResourceByName(name='CL_nom'):
  if name not in resources:
    print('Name must be: \'' + '\', or \''.join(list(resources.keys())) + '\'')
    resources[name]
  else:
    resource = copy.deepcopy(resources[name])
    return resource

def main(argv):
  del argv  # Unused.


if __name__ == '__main__':
  app.run(main)
