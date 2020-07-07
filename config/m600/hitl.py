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

"""Options related to HITL."""

from makani.avionics.network import aio_labels
from makani.config import mconfig
from makani.control import system_types


@mconfig.Config
def MakeParams():
  servo_levels = [system_types.kActuatorHitlLevelSimulated
                  for _ in range(aio_labels.kNumServos)]

  return {
      'config': {
          # HITL level of the simulator.
          #
          # The simulator and controller are responsible for overriding this at
          # runtime prior to handshakes.
          'sim_level': system_types.kSimulatorHitlLevelNone,

          # Whether to use the software joystick.
          'use_software_joystick': False,

          # HITL level of GS02.
          'gs02_level': system_types.kActuatorHitlLevelSimulated,

          # Number of seconds the simulator will tolerate not receiving an
          # update from GS02 before exiting.
          'gs02_timeout_sec': 1.0,

          # HITL level of all motors.
          'motor_level': system_types.kActuatorHitlLevelSimulated,

          # Number of seconds the simulator will tolerate not receiving an
          # update from a HITL motor before exiting.
          'motor_timeout_sec': 1.0,

          # Whether the simulator should send dyno commands.
          'send_dyno_commands': False,

          # HITL levels of servos, by label.
          'servo_levels': servo_levels,

          # Number of seconds the simulator will tolerate not receiving an
          # update from a HITL servo before exiting.
          'servo_timeout_sec': 1.0,

          # HITL level of tether release.
          'tether_release_level': system_types.kActuatorHitlLevelSimulated,

          # Number of seconds the simulator will tolerate not receiving an
          # update from a HITL loadcell (for tether release) before exiting.
          'tether_release_timeout_sec': 1.0,
      }
  }
