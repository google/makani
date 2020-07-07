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

"""Motor parameters."""

from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      # Maximum modulation usage.
      # The 0.97 comes from dead time limits in the SVPWM algorithm
      # and reflects 1 usec per transition at 15 kHZ as expected for
      # Ozone controller and the 0.95 comes from a flux weakening margin.
      # Dead time effect is calculated in svpwm.c line 55.
      # (1.0f - 2.0f * MIN_GATE_HOLD_TIME / pwm_period)
      'modulation_limit': 0.97 * 0.95,

      # Motor phase current limit [A (phase peak)].
      'phase_current_cmd_limit': 250.0,

      # Motor quadrature current command limits [A (phase peak)].
      'iq_cmd_lower_limit': -250.0,
      'iq_cmd_upper_limit': 250.0,

      # Yasa parameters for 2.3 motor come from:
      #   YASA Motors Engineering Report No. 2018 (2016-09-08),
      #   "Makani Gen 2.3 Key Motor Parameters".
      #
      # Note that the YASA document reports current as [Amps (RMS)] while we
      # report motor phase (and d-q) current as [Amps (peak)].

      # Yasa d and q-axis inductances [H].
      # These numbers are poorly known (especially the d-axis
      # inductance) and should be re-examined at some
      # point. However they have been tweaked assuming a
      # non-salient PMSM so that the shaft power is limited to 102
      # kW to match experimental measurements. Also note that
      # saliency is not supported in the sim at this point.
      'Ld': 0.9e-3,
      'Lq': 0.9e-3,

      # Yasa phase resistance [Ohm].
      # Fitted thermal model assuming steady state heating of wire
      # gives Rs = phase_rms^2 * 5.82716e-7 + 0.0776.
      # Another sensible approach that we'll use here is to overestimate the
      # resistance using max wire temperature of 160C which will both over-
      # estimate the loss and be more conservative when calculating limits.
      # R = R_20 * (1 + alpha_Cu * (Tmax - 20)).
      'Rs': 0.067 * (1 + 0.004041 * (160 - 20)),

      # Yasa flux linkage [N-m/A] = [V-s/rad] = [Wb] per pole.
      # The amplitude invariant Park transform is used to define
      # the phase current such that this constant needs to be
      # multiplied by a factor of 3/2 in addition to iq and npp to
      # get torque (assuming a non-salient machine). An additional
      # factor of 0.9 is applied to account for non-linearity at
      # higher torque.
      'flux_linkage': 0.1666 * 0.9,

      # Number of Yasa rotor pole pairs [#].
      'num_pole_pairs': 15,

      # From loss modeling work.  Fitted from 1200 V data at [redacted].

      # Hysteresis loss [W/A^2/(rad/sec)^2] is phase_current^2 * w^2 * coeff.
      # From fit model coeff = 1/540000.
      'hysteresis_loss_coefficient': 0.000001851851852,

      # Speed dependent losses fitted from near zero torque curves
      'omega_loss_coefficient_cubic': 0.00008373,
      'omega_loss_coefficient_sq': 0.009943,
      'omega_loss_coefficient_lin': 2.347,

      # Motor controller specific loss parameters.  Per phase.
      'rds_on': 0.003625,

      # This one is messy.  Double pulse testing gives us specific loss
      # [J/(V*A)] equal to 42e-9 for turn on and 56e-9 for turn off.  This is
      # per switching cycle and per module and assumes instantaneous current.
      # This didn't separate out the fixed loss for switching the output
      # capacitance of the modules.  To improve this, fixed losses are
      # subtracted from a high load point and then added in separately.  This is
      # a fudge.  Fixed losses also come from the above spreadsheet.  400 A,
      # 1500V used as the point for calculating the effective specific_switching
      # loss.
      'specific_switching_loss': 75.3e-9,

      # These are per module and per switch so need to be multiplied by 15000
      # and 3 to match zero current switching losses from spreadsheet.
      'fixed_loss_sq_coeff': 3.8e-9,
      'fixed_loss_lin_coeff': 3.372e-6,
      'switching_frequency': 15000.0
  }
