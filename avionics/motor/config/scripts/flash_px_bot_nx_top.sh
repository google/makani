#!/bin/bash -e
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

#
# Config a set of stacked Yasa motors for +x spin on the bottom and -x on top.
#
# Usage: ./flash_px_bot_nx_top.sh
#
# Configure a set of stacked motors to have the nominal spin directions shown
# below when viewed from the front. These spin directions should induce swirl
# into the cooling ducts without modification.
#
#        STO STI PTI PTO        M600a
#         x   x | x   x         Front
#  -------|---|-o-|---|-------  View
#         .   .   .   .
#        SBO SBI PBI PBO

# Bottom.
program motor_sbo --config yasa_swap_none_prop_rev2_positive_stacked
program motor_sbi --config yasa_swap_none_prop_rev2_positive_stacked
program motor_pbi --config yasa_swap_none_prop_rev2_positive_stacked
program motor_pbo --config yasa_swap_none_prop_rev2_positive_stacked

# Top.
program motor_pto --config yasa_swap_none_prop_rev1_negative_stacked
program motor_pti --config yasa_swap_none_prop_rev1_negative_stacked
program motor_sti --config yasa_swap_none_prop_rev1_negative_stacked
program motor_sto --config yasa_swap_none_prop_rev1_negative_stacked

