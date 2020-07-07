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

"""All parameters."""
from makani.config import mconfig


@mconfig.Config(deps={
    'control': 'common.control.control_params',
    'monitor': 'common.monitor.monitor_params',
    'sim': 'common.sim.sim_params',
    'system': mconfig.WING_MODEL + '.system_params'
})
def MakeParams(params):
  return {
      'control': params['control'],
      'monitor': params['monitor'],
      'sim': params['sim'],
      'system': params['system']
  }
