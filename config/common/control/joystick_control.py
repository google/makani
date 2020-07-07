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

"""Joystick control parameters."""
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      # Throttle above ascend/payout threshold will switch to a
      # ascend/payout sequence.  Throttle below descend/reel-in
      # threshold will switch to a reel-in and descend sequence.
      # Throttle below prep-trans-out throttle will force
      # kFlightModeCrosswindPrepTransOut.
      'ascend_payout_throttle': 0.8,
      'return_to_crosswind_throttle': 0.8,
      'prep_trans_out_throttle': 0.5,
      'descend_reel_in_throttle': 0.4,

      # Throttle position below which the rotors are commanded to idle and the
      # winch speed is set to zero.
      'e_stop_throttle': 0.08,

      # Time [s] required below the e-stop throttle level after which
      # various actions are latched.  In particular, this is used to
      # enter the perched state or force a slow ramp of the motors in
      # the hover controller.
      'e_stop_throttle_latch_time': 2.0
  }
