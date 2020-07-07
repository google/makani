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

"""Simulated ground-frame parameters."""

from makani.config import mconfig


@mconfig.Config(
    deps={'test_site': 'common.test_site',
          'ground_frame': 'base_station.ground_frame'})
def MakeParams(params):

  return {
      # Position [m] of the origin of the ground frame in ECEF.
      # TODO: Could we eliminate this parameter entirely?
      'pos_ecef': params['ground_frame']['origin_ecef']
  }
