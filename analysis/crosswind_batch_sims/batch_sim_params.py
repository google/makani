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

"""Scoring functions used by this batch simulation."""

from makani.config import mconfig
from makani.control import system_types
from makani.lib.python import c_helpers
from makani.lib.python import wing_flag
from makani.lib.python.batch_sim import scoring_functions
from makani.lib.python.batch_sim.scoring_functions import aero
from makani.lib.python.batch_sim.scoring_functions import buoy
from makani.lib.python.batch_sim.scoring_functions import crosswind
from makani.lib.python.batch_sim.scoring_functions import gs02
from makani.lib.python.batch_sim.scoring_functions import hover
from makani.lib.python.batch_sim.scoring_functions import kinematics
from makani.lib.python.batch_sim.scoring_functions import loads
from makani.lib.python.batch_sim.scoring_functions import power
from makani.lib.python.batch_sim.scoring_functions import rotors
from makani.lib.python.batch_sim.scoring_functions import software
from makani.lib.python.batch_sim.scoring_functions import tether
from makani.lib.python.batch_sim.scoring_functions import trans_in
import numpy as np

_FLIGHT_PLAN_HELPER = c_helpers.EnumHelper('FlightPlan', system_types,
                                           prefix='kFlightPlan')
_WING_MODEL_HELPER = c_helpers.EnumHelper('WingModel', system_types,
                                          prefix='kWingModel')


def LimitsHelper(limits, offsets):
  """Add offsets to lower/upper limit params for passing to scoring functions.

  Args:
    limits: Iterable of len 1 or 2 that contains the lower and/or upper limit.
    offsets: Iterable, twice the length of limits, that will be added to the
      limits.
  Returns:
    good_bad_limits: List of the limit values with the offsets applied.
  Example:
    LimitsHelper([110., 190.], [5., 10., -10., -5.])
    returns: [115., 120., 180., 185.]
  """

  assert len(limits) == len(offsets)/2, 'Check input types, see doc string.'
  good_bad_limits = []
  for ii, limit in enumerate(limits):
    good_bad_limits.append(limit + offsets[0 + 2*ii])
    good_bad_limits.append(limit + offsets[1 + 2*ii])
  return good_bad_limits


class CrosswindSweepsParameters(object):
  """Common parameters to the client and worker for this batch sim.

  Attributes:
    scoring_functions: List of scoring functions.
  """

  def __init__(self, only_steady_flight=False, steady_flight_mode_time=20,
               flight_plan='kFlightPlanTurnKey', offshore=False,
               wing_model=system_types.kWingModelYm600):
    """Initalize the crosswind batch sim scoring functions.

    Args:
      only_steady_flight: Boolean that describes whether the test should only
        be run in steady crosswind flight, or whether it should include the
        transient from transition-in.  Note that some scores, such as airspeed
        and radius error, are only applied in steady crosswind in either case.
      steady_flight_mode_time: Time [s] after which transients from
        transition-in have gone away.  This may be used to test the performance
        of the crosswind controller under steady conditions.
      flight_plan: String or Enum that defines the flight plan used in a
        given batch simulation run.
      offshore: Flag indicating an offshore scenario.
      wing_model: String or Enum that defines the wing model to score.
    """

    flight_plan = _FLIGHT_PLAN_HELPER.ShortName(flight_plan)
    wing_config_name = wing_flag.WingModelToConfigName(wing_model)
    assert wing_config_name in ['m600', 'oktoberkite']

    # Time [s] to end simulation.
    if flight_plan not in ['TurnKey', 'HighHover']:
      assert False, 'Unsupported flight plan: %s' % flight_plan

    # Flight mode time threshold [s] used by default.
    common_flight_mode_time_threshold = (steady_flight_mode_time
                                         if only_steady_flight else 0.0)

    mconfig.WING_MODEL = wing_config_name

    # TODO: Allow the score limits to depend on test site.
    overrides = {
        'system': {
            'flight_plan': _FLIGHT_PLAN_HELPER.Value(flight_plan)
        }
    }
    params = mconfig.MakeParams('common.all_params',
                                overrides=overrides,
                                override_method='derived')

    limits = params['system']['limits']

    # TODO: Move params only needed for scoring functions out of
    # the main config files (wing.py, crosswind.py, etc) and to
    # separate, dedicated scoring config file.
    if wing_model == 'm600':
      # TODO: Move these params to the config once values for
      # Oktoberkite are set.
       # For the MaxTailMomentScoringFunction scoring function.
      tail_moment_limits = {
          'x': [-17.352, -14.341, 14.341, 17.352],
          'y': [-74.250, -61.364, 53.760, 65.013],
          'z': [-81.675, -67.500, 67.500, 81.675],
      }

    self.scoring_functions = (
        # Rubric for assigning severity is as follows:
        # 0- Informational scores, don't contribute to aggregate score.
        # 1- Minimal (or negligible) impact to mission or performance
        #    (~3% performance reduction).
        # 2- Small reduction in mission return or performance
        #    (~10% performance reduction).
        # 3- Moderate reduction in mission return or performance
        #    (~30% performance reduction).
        # 4- Significant reduction in mission return or performance
        #    (~60% performance reduction). Potentially indicating end of flight.
        # 5- Mission failure/vehicle loss/ground station loss.
        #    All crash scoring functions are assigned severity=5.

        ###########################################
        # Crash-specific scoring functions.
        ###########################################

        # Min height above ground level. Checks for basic condition of kite
        # making contact with the ground in all modes of flight. The tail is
        # expected to be 3.2 meters above the ground when the kite is on the
        # perch. We set the "good" limit to 2.0 meters. The bad limit is
        # equivalent to zero clearance to the ground indicating contact.
        # The sphere of contact is defined by a radius from the kite body origin
        # to the center of pressure of the elevator (or to the tip of the tail
        # spike, if it exists). This does not fully envelop the wing tips, but
        # it is assumed unlikely that a wingtip can contact the ground without
        # subsequent reduced clearance that would be captured by this function
        # (e.g. an uncontrolled descent).
        scoring_functions.CrashScoringFunction(
            kinematics.KiteHeightAglScoringFunctionWrapper(
                kinematics.MinKiteHeightAglScoringFunction(2.0, 0.0,
                                                           severity=5))),

        # Airspeed at the rotors should maintain margin from the vortex ring
        # state (VRS) threshold. Each rotor is scored from 0 (not at risk of
        # VRS) to 1 (fully in VRS) and all rotors are summed. Nearly half the
        # rotors in VRS (3.5/8) or all rotors partially in VRS shall be the
        # limit, and non-zero values begin when any individual rotor is at
        # risk of VRS.
        scoring_functions.CrashScoringFunction(
            rotors.VortexRingStateScoringFunction(0.0, 3.5, severity=5)),

        # Breaker applies a 30 minute thermal time constant.
        # 225 amps is 100% breaker rating.
        # Discussion here: https://goo.gl/NfkRMi
        scoring_functions.CrashScoringFunction(
            power.BreakerCurrentScoringFunction(0.8 * 225.0, 225.0,
                                                severity=5)),

        # Maximum power motoring [kW].  900 can be sustained for
        # 3 hours based on breaker rating.  1120 kW can be sustained
        # for 30 minutes.  600 seconds for 1.25 * rating.
        scoring_functions.CrashScoringFunction(
            power.PowerConsumedMaxScoringFunction(1120.0, 1400.0, severity=5)),

        # Maximum power generated [kW].
        # The load bank can sink up to 1.5 MW peak (up to an estimated 240
        # seconds) and 1.0 MW continuously. This limit is set by the 1200A
        # 480VAC load bank breaker.
        # We try to maintain at least 300 kW of load on the Aggreko at all times
        # so the loadbank will attempt to dissipate 300 kW plus any power
        # generated by the kite. Analysis of crosswind flights in PR shows that
        # the loadbank tracking is good enough to maintain at least 200 kW of
        # load on the Aggreko except when power transients exceed 1 MW/s.
        # 950 kW of generation (1.25 MW load bank dissipation) can be sustained
        # for an estimated 300 seconds. Limit peak generation to 1150 kW.
        # These limits assume that the kite will not generate an average power
        # of more than 600 kW. If this changes, we'll need to upgrade the
        # load bank breaker rating or reduce the Aggreko minimum load.
        # TODO(b/146362655): Add load bank breaker current scoring function.
        scoring_functions.CrashScoringFunction(
            power.PowerGeneratedMaxScoringFunction(950.0, 1150.0, severity=5)),

        # Maximum rate of power change [kW/sec].
        # 500 kW / sec slew rate was the request to the controls team.
        # 1000.0 kW / sec was about what we saw during the crash.
        # The microgrid has been hardened significantly, but testing is
        # limited due to lack of hardware and money to build a fully rated
        # power simulation.  For now, the upper limit is set at the crash
        # value since we don't have the ability to validate a higher limit.
        scoring_functions.CrashScoringFunction(
            power.PowerTransientScoringFunction(500.0, 1000.0, severity=5)),
    )

    if wing_model == 'm600':
      # Max allowable combination of aerodynamic lift and inertial loads in
      # kite -Z direction. Returns non-dimensional failure index based on an
      # aerodynamic lift limit between 258 and 500 kN (based on HALB & THT
      # wing design load cases https://goo.gl/ePwZ7S), with knockdown for
      # roll acceleration that causes a change in lift distribution that
      # increases bending moments in the center of the wing and a kite body
      # acceleration limit of 85.9 m/s^2 (based on RPX-09 max kite body
      # acceleration at time of mid-air breakup). A value of 1.0 indicates
      # structural failure; limits of 0.608 & 0.733 are limit load- and proof
      # load-equivalent failure indexes. Source derivation for limits and
      # interaction equation here: https://goo.gl/qUjCgy
      self.scoring_functions += (
          scoring_functions.CrashScoringFunction(
              loads.MaxWingBendingFailureIndex(
                  0.608, 0.733, severity=5,
                  aero_tension_limit=500e3,
                  accel_limit=85.9,
                  domega_limit=5.79)),
      )

      # Maximum wing/fuselage junction moment [N-m]. The limits were set
      # during the integrated fuselage proof-test (https://goo.gl/YDzcj9).
      # -My limits were determined by FEA of the LAHB case with My reversed
      # (-207 kN-m) multiplying by the 1st applicable eigenvalue (0.45) and
      # dividing by a safety factor of 1.518 (buckling, extreme)
      # (https://goo.gl/JXGWLR) = 61.364 kN-m The good limits are the limit
      # loads and the bad limits are the proof loads (kN-m).
      for ax, lims in tail_moment_limits.items():
        self.scoring_functions += (
            scoring_functions.CrashScoringFunction(
                loads.MaxTailMomentScoringFunction(ax, *lims, severity=5)),
        )

    self.scoring_functions += (
        # Maximum servo hinge moments [N-m] should be
        # less than 140 N-m and should never exceed 176.0 N-m which would
        # damage any one actuator. Limits are from servo tests shown here:
        # https://goo.gl/aoWts4
        scoring_functions.CrashScoringFunction(
            loads.MaxServoMoment(140.0, 176.0, severity=5)),

        # Maximum rotor resultant in-plane moment [N-m].
        # It should not exceed 1230 N-m before the pylons may break. Non-zero
        # values shall begin at 1016 N-m which is 1230 / (1.1 * 1.1) to account
        # for the safety factors on this value.
        # TODO(b/124793226): The sim commonly reports rotor in-plane moments
        # greater than 4000 N-m which, according to the limits of this scoring
        # function, would break the pylons. Investigate if the limits are
        # over-conservative or if the scoring function is incorrect.
        scoring_functions.CrashScoringFunction(
            loads.MaxRotorInPlaneMoment(1016.0, 1230.0, severity=5)),

        # At the Parker Ranch test site, the command center is located at
        # 45 deg azimuth. We do not want to fly within 45 deg of this
        # azimuth. Bad scores start in this zone, and are full red
        # when >=5 deg inside this zone.
        # For offshore sims, we do not evaluate azi no-go scores, and handle
        # this risk through operational procedures.
        # b/137571714
        scoring_functions.CrashScoringFunction(
            scoring_functions.PayoutFilter(
                kinematics.AzimuthDegNoGoZoneScoringFunction(
                    0.0, 90.0, 5.0, -180.0, 180.0, severity=5),
                min_payout=50.0)),

        # The tether elevation is -7.8 deg right before ascend.
        # The kite can ascend by 3.0 m on the panel; however, with a steady
        # state hover attitude, the kite has to pitch back by 24 deg, leading to
        # tail-side fuselage hitting the panel. Analysis shows that with a perch
        # peg that is compressed by 30%, the maximum ascent distance on the
        # panel without the risk of hitting the fuselage is 1.6 m.
        # Given that the length of the boom is 7.892 m, the maximum increase in
        # tether elevation during HoverAscend is 11.6 deg. This means a nominal
        # range of [-7.8, 3.8] deg. For the upper bound, the good/bad limits are
        # softened to [2.8, 4.8] deg. For the lower bound, the good/bad limits
        # are softened to [-9.8, -7.8] deg to account for false positives
        # introduced by sim's contactor modeling. See b/137197170.
        scoring_functions.CrashScoringFunction(
            scoring_functions.FlightModeFilter(
                'kFlightModeHoverAscend',
                hover.TetherElevationScoringFunction(
                    -9.8, -7.8, 2.8, 4.8, severity=5,
                    transform_stages=None,
                    sustained_duration=None))),

        # After nominal perching, the tether elevation sometimes goes to
        # -8.8 deg. It is different from ascent likely due to the contactor
        # interaction modeling in the sim. With the same tether elevation
        # window size as ascent, the nominal range is then [-8.8, 2.8]. The
        # limits are then softened following the same logic as in ascent.
        scoring_functions.CrashScoringFunction(
            scoring_functions.FlightModeFilter(
                'kFlightModeHoverDescend',
                hover.TetherElevationScoringFunction(
                    -10.8, -8.8, 1.8, 3.8, severity=5,
                    transform_stages=None,
                    sustained_duration=None))),

        scoring_functions.CrashScoringFunction(
            scoring_functions.PerchingFilter(
                hover.TetherElevationScoringFunction(
                    -10.8, -8.8, 1.8, 3.8, severity=5,
                    transform_stages=None,
                    sustained_duration=None))),

        scoring_functions.CrashScoringFunction(
            scoring_functions.PerchingFilter(
                hover.PanelAzimuthTrackingScoringFunction(
                    84.55, 86.55, 90.47, 92.47, severity=5))),

        # The near perch tether elevation window should be the same as the
        # ascent/descent tether elevation window, to make sure the kite does
        # not hit the panel, and the peg is able to engage the panel if the
        # kite reels in.
        scoring_functions.CrashScoringFunction(
            scoring_functions.PayoutFilter(
                hover.TetherElevationScoringFunction(
                    -9.8, -7.8, 13.0, 15.0, severity=5, transform_stages=None,
                    sustained_duration=0.5),
                min_payout=0.0, max_payout=6.0, flight_mode_filter='PayOut')),

        scoring_functions.CrashScoringFunction(
            scoring_functions.PayoutFilter(
                hover.TetherElevationScoringFunction(
                    -10.8, -8.8, 12.0, 14.0, severity=5, transform_stages=None,
                    sustained_duration=0.5),
                min_payout=0.0, max_payout=6.0, flight_mode_filter='ReelIn')),

        # Tether elevation limits in reel modes. See b/117169069,
        # b/117995426, and b/123407643 for details on the limits and
        # design of this scoring function.
        scoring_functions.CrashScoringFunction(
            scoring_functions.PayoutFilter(
                hover.TetherElevationScoringFunction(
                    -2.0, 1.0, 12.0, 18.0, severity=5, transform_stages=None,
                    sustained_duration=0.5),
                min_payout=6.0, flight_mode_filter='PayOut')),
        scoring_functions.CrashScoringFunction(
            scoring_functions.PayoutFilter(
                hover.TetherElevationScoringFunction(
                    -2.0, 1.0, 12.0, 18.0, severity=5, transform_stages=None,
                    sustained_duration=0.5),
                min_payout=6.0, flight_mode_filter='ReelIn')),

        # Tether elevation should be well-controlled during the transforms.
        # Note that when transforming from high-tension to reel mode, the
        # transform stages proceed in the sequence of [0, 1, 2, 3, 4]. The
        # sequence is [0, 4, 3, 2, 1] when transforming from reel to
        # high-tension mode.
        #
        # When transforming from high-tension to reel, the tether
        # elevation should fall within [0, 17] deg before and during the azimuth
        # slew (stages 1 and 2) to avoid touching the levelwind or its bump
        # block and the panel. Conventionally, there is a +/- 2 deg of margin
        # (1 deg for encoders and 1 for extra safety), therefore the new window
        # for tranform is [2, 15].
        # The limits in stage 4 are the same as reel.
        # Stage 3 is the transition stage between 2 and 4.
        #
        # References:
        #   go/makani-tether-elevation-control.
        #   go/makani-levelwind-bumpblock.

        # During stage 1, the drum rotates to move the GSG from the
        # upper position to the lower position.  Because the tether is
        # on the GSG at this point, tether elevation control is not
        # critical.
        scoring_functions.FlightModeFilter(
            'kFlightModeHoverTransformGsUp',
            hover.TetherElevationScoringFunction(
                2.0, 3.0, 14.0, 15.0, severity=1,
                transform_stages=[1],
                extra_system_labels=['experimental'])),

        scoring_functions.CrashScoringFunction(
            scoring_functions.FlightModeFilter(
                'kFlightModeHoverTransformGsUp',
                hover.TetherElevationScoringFunction(
                    2.0, 3.0, 14.0, 15.0, severity=5,
                    transform_stages=[2]))),

        scoring_functions.CrashScoringFunction(
            scoring_functions.FlightModeFilter(
                'kFlightModeHoverTransformGsUp',
                hover.TetherElevationScoringFunction(
                    2.0, 3.0, 12.0, 18.0, severity=5,
                    transform_stages=[3]))),

        scoring_functions.CrashScoringFunction(
            scoring_functions.FlightModeFilter(
                'kFlightModeHoverTransformGsUp',
                hover.TetherElevationScoringFunction(
                    -2.0, 1.0, 12.0, 18.0, severity=5,
                    transform_stages=[0, 4]))),

        scoring_functions.FlightModeFilter(
            'kFlightModeHoverTransformGsDown',
            hover.TetherElevationScoringFunction(
                2.0, 3.0, 14.0, 15.0, severity=0,
                transform_stages=[0, 1],
                extra_system_labels=['experimental'])),

        scoring_functions.CrashScoringFunction(
            scoring_functions.FlightModeFilter(
                'kFlightModeHoverTransformGsDown',
                hover.TetherElevationScoringFunction(
                    2.0, 3.0, 14.0, 15.0, severity=5,
                    transform_stages=[2]))),

        scoring_functions.CrashScoringFunction(
            scoring_functions.FlightModeFilter(
                'kFlightModeHoverTransformGsDown',
                hover.TetherElevationScoringFunction(
                    2.0, 3.0, 12.0, 18.0, severity=5,
                    transform_stages=[3]))),

        scoring_functions.CrashScoringFunction(
            scoring_functions.FlightModeFilter(
                'kFlightModeHoverTransformGsDown',
                hover.TetherElevationScoringFunction(
                    -2.0, 1.0, 12.0, 18.0, severity=5,
                    transform_stages=[4]))),
    )

    if flight_plan == 'TurnKey':

      # We apply a crosswind filter to the MaxWingBendingFailureIndex and
      # MaxTailMoments scoring function to help isolate issues occurring
      # during the crosswind flight modes from the ones occurring during
      # hover.
      # NOTE: Since we expect these scoring functions to
      # trigger during crosswind, the present implementation makes it
      # difficult to isolate bad events occurring in hover in the same log.
      if wing_model == 'm600':
        self.scoring_functions += (
            scoring_functions.CrashScoringFunction(
                scoring_functions.FlightModeFilter(
                    ['kFlightModeCrosswindNormal',
                     'kFlightModeCrosswindPrepTransOut'],
                    loads.MaxWingBendingFailureIndex(
                        0.608, 0.733, severity=5,
                        aero_tension_limit=500e3,
                        accel_limit=85.9,
                        domega_limit=5.79),
                    filter_name_prefix='Crosswind - ')),
        )
        for ax, lims in tail_moment_limits.items():
          self.scoring_functions += (
              scoring_functions.CrashScoringFunction(
                  scoring_functions.FlightModeFilter(
                      ['kFlightModeCrosswindNormal',
                       'kFlightModeCrosswindPrepTransOut'],
                      loads.MaxTailMomentScoringFunction(
                          ax, *lims, severity=5),
                      filter_name_prefix='Crosswind - ')),
          )

      self.scoring_functions += (
          # The tether must maintain some minimum height [m] off of the ground
          # (minimum height is 1 meter AGL to account for bushes). We restrict
          # this scoring function to high tension modes and we exclude Crosswind
          # for which the kite height AGL serves as a proxy.
          # NOTE: This scoring function assumes that the terrain at the test
          # site is flat (see b/70640378#comment8)
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeHoverFullLength',
                  tether.TetherHeightAglMinScoringFunction(4.0, 1.0,
                                                           severity=5))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  ['kFlightModeHoverTransOut',
                   'kFlightModeHoverPrepTransformGsDown'],
                  tether.TetherHeightAglMinScoringFunction(4.0, 1.0,
                                                           severity=5),
                  filter_name_prefix='HoverTransOut')),

          # The kite must not deviate too far from the tether sphere during
          # CrosswindNormal (see analysis based on RPX-06, 07, 08 and 09:
          # b/78361234).
          # The initial transients following TransIn are ignored.
          scoring_functions.CrashScoringFunction(
              scoring_functions.CrosswindFilter(
                  kinematics.TetherSphereDeviationScoringFunction(
                      2.5, 5.0, severity=5, mean_tether_tension=100e3),
                  flight_mode_time_threshold=steady_flight_mode_time)),

          # Tether pitch scoring functions with high tension threshold, used to
          # evaluate how close the bridles are to a collision with the pylons.
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeTransIn',
                  tether.PitchDegScoringFunction(
                      *LimitsHelper(
                          np.rad2deg(limits['tether_hover_pitch_rom']),
                          [5.0, 10.0, -2.0, -1.0]),
                      severity=5,
                      tension_threshold=(
                          limits['tether_pitch_tension_threshold'])))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.CrosswindFilter(
                  tether.PitchDegScoringFunction(
                      *LimitsHelper(
                          np.rad2deg(limits['tether_crosswind_pitch_rom']),
                          [5.0, 10.0, -2.0, -1.0]),
                      severity=5,
                      tension_threshold=(
                          limits['tether_pitch_tension_threshold'])))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeCrosswindPrepTransOut',
                  tether.PitchDegScoringFunction(
                      *LimitsHelper(
                          np.rad2deg(limits['tether_crosswind_pitch_rom']),
                          [5.0, 10.0, -2.0, -1.0]),
                      severity=5,
                      tension_threshold=(
                          limits['tether_pitch_tension_threshold'])))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeHoverTransOut',
                  tether.PitchDegScoringFunction(
                      *LimitsHelper(
                          np.rad2deg(limits['tether_hover_pitch_rom']),
                          [5.0, 10.0, -2.0, -1.0]),
                      severity=5,
                      tension_threshold=(
                          limits['tether_pitch_tension_threshold'])))),

          # Tether roll scoring functions with low tension threshold, used to
          # evaluate how close the system is to a collapse of the bridle point.
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeTransIn',
                  tether.RollDegScoringFunction(
                      *LimitsHelper(
                          np.rad2deg(limits['tether_hover_roll_rom']),
                          [5.0, 15.0, -15.0, -5.0]),
                      severity=5,
                      tension_threshold=1000.0))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.CrosswindFilter(
                  tether.RollDegScoringFunction(
                      *LimitsHelper(
                          np.rad2deg(limits['tether_crosswind_roll_rom']),
                          [5.0, 15.0, -15.0, -5.0]),
                      severity=5,
                      tension_threshold=1000.0))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeCrosswindPrepTransOut',
                  tether.RollDegScoringFunction(
                      *LimitsHelper(
                          np.rad2deg(limits['tether_crosswind_roll_rom']),
                          [5.0, 15.0, -15.0, -5.0]),
                      severity=5,
                      tension_threshold=1000.0))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  ['kFlightModeHoverTransOut',
                   'kFlightModeHoverPrepTransformGsDown'],
                  tether.RollDegScoringFunction(
                      *LimitsHelper(
                          np.rad2deg(limits['tether_hover_roll_rom']),
                          [5.0, 15.0, -15.0, -5.0]),
                      severity=5,
                      tension_threshold=1000.0),
                  filter_name_prefix='HoverTransOut')),

          # Max airspeed [m/s] allowed during steady crosswind. This
          # score should only be applied after the transient from
          # transition-in has died.
          scoring_functions.CrashScoringFunction(
              scoring_functions.CrosswindFilter(
                  aero.AirspeedMaxScoringFunction(
                      *LimitsHelper(
                          [params['control']['crosswind']['power'][
                              'max_airspeed']],
                          [-10.0, -5.0]),
                      severity=5),
                  flight_mode_time_threshold=steady_flight_mode_time)),

          # Maximum tension [kN] that the tether is allowed to see
          # during crosswind flight is 234 kN, based on http://b/64933954.
          # The crash criteria "good" value corresponds to the red
          # indicator limit on the flight test monitors (234 kN) and the "bad"
          # value corresponds to the tether proof load (263 kN).
          scoring_functions.CrashScoringFunction(
              scoring_functions.CrosswindFilter(
                  tether.TensionMaxScoringFunction(234.0, 263.0, severity=5))),

          # Rotor max advance ratio margin. When negative, the max instantaneous
          # advance ratio of one (or more) rotors exceeded the limit, which
          # means the rotor is stalled according to the rotor tables. The bad
          # score is set slightly negative so small fluctuations are not
          # considered a crash.
          scoring_functions.CrashScoringFunction(
              scoring_functions.CrosswindFilter(
                  rotors.RotorStallScoringFunction(0.0, -0.05, severity=5))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeCrosswindPrepTransOut',
                  rotors.RotorStallScoringFunction(0.0, -0.05, severity=5))),

          # Ground-side Gimbal (GSG) Yoke and Termination angles [deg]. These
          # angles are physically limited by GSG geometry as referenced in
          # b/78529394. The gsg yoke was designed to have a Range of Motion of
          # +/-105 degrees (see https://goo.gl/kWMBRz) which was later reduced
          # to +/-73 degrees with the introduction of MV cables and slip ring
          # hardware. A "guard band" of ~10% is included with these functions
          # to warn of flight conditions where GSG limits are almost exceeded.
          # TODO: Use a drum angle filter for GSG angles.
          # See b/138462894 for more details.
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeHoverTransformGsUp',
                  scoring_functions.Gs02TransformStageFilter(
                      1,
                      gs02.GsgYokeAnglesScoringFunction(
                          -73.0, -65.0, 65.0, 73.0, severity=5)))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeHoverTransformGsUp',
                  scoring_functions.Gs02TransformStageFilter(
                      [1, 2, 3],
                      gs02.GsgTerminationAnglesScoringFunction(-34.0, -31.0,
                                                               31.0, 34.0,
                                                               severity=5)))),

          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  ['kFlightModeHoverFullLength',
                   'kFlightModeHoverAccel'],
                  gs02.GsgYokeAnglesScoringFunction(-73.0, -65.0, 65.0, 73.0,
                                                    severity=5))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  ['kFlightModeHoverFullLength',
                   'kFlightModeHoverAccel'],
                  gs02.GsgTerminationAnglesScoringFunction(-34.0, -31.0,
                                                           31.0, 34.0,
                                                           severity=5))),

          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeTransIn',
                  gs02.GsgYokeAnglesScoringFunction(-73.0, -65.0, 65.0, 73.0,
                                                    severity=5))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeTransIn',
                  gs02.GsgTerminationAnglesScoringFunction(-34.0, -31.0,
                                                           31.0, 34.0,
                                                           severity=5))),

          scoring_functions.CrashScoringFunction(
              scoring_functions.CrosswindFilter(
                  gs02.GsgYokeAnglesScoringFunction(-73.0, -65.0, 65.0, 73.0,
                                                    severity=5),
                  flight_mode_time_threshold=
                  common_flight_mode_time_threshold)),
          scoring_functions.CrashScoringFunction(
              scoring_functions.CrosswindFilter(
                  gs02.GsgTerminationAnglesScoringFunction(-34.0, -31.0,
                                                           31.0, 34.0,
                                                           severity=5),
                  flight_mode_time_threshold=
                  common_flight_mode_time_threshold)),

          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeCrosswindPrepTransOut',
                  gs02.GsgYokeAnglesScoringFunction(-73.0, -65.0, 65.0, 73.0,
                                                    severity=5))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeCrosswindPrepTransOut',
                  gs02.GsgTerminationAnglesScoringFunction(-34.0, -31.0,
                                                           31.0, 34.0,
                                                           severity=5))),

          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  ['kFlightModeHoverTransOut',
                   'kFlightModeHoverPrepTransformGsDown'],
                  gs02.GsgYokeAnglesScoringFunction(-73.0, -65.0, 65.0, 73.0,
                                                    severity=5),
                  filter_name_prefix='HoverTransOut')),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  ['kFlightModeHoverTransOut',
                   'kFlightModeHoverPrepTransformGsDown'],
                  gs02.GsgTerminationAnglesScoringFunction(-34.0, -31.0,
                                                           31.0, 34.0,
                                                           severity=5),
                  filter_name_prefix='HoverTransOut')),

          # TODO: Use a drum angle filter for GSG angles.
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeHoverTransformGsDown',
                  scoring_functions.Gs02TransformStageFilter(
                      [0, 1],
                      gs02.GsgYokeAnglesScoringFunction(
                          -73.0, -65.0, 65.0, 73.0, severity=5)))),
          scoring_functions.CrashScoringFunction(
              scoring_functions.FlightModeFilter(
                  'kFlightModeHoverTransformGsDown',
                  scoring_functions.Gs02TransformStageFilter(
                      [0, 1, 2, 3],
                      gs02.GsgTerminationAnglesScoringFunction(-34.0, -31.0,
                                                               31.0, 34.0,
                                                               severity=5)))),

          # Relative to the nominal kite perched position (perched_wing_pos_p),
          # the kite can translate 0.335 m or 0.755 m towards the port or
          # starboard sides, respectively, before falling-off the perch panel
          # (see b/137225712).
          # Adding a 4.5 cm margin to account for radial position change, the
          # kite will always be on the perch if the difference between launch
          # and land positions is less than 0.38 m (good value), and it will be
          # always off the perch if it is greater than 0.80 m (bad value).
          scoring_functions.CrashScoringFunction(
              hover.PerchedPositionScoringFunction(
                  0.38, 0.80, severity=5)),

          # Max tether twists, in continuous number of revolutions.
          gs02.GSTetherTwistScoringFunction(-10.0, -2.0, 2.0, 10.0, severity=4),

      )

    self.scoring_functions += (

        ###########################################
        # Hover scoring functions.
        ###########################################
        # TODO: Add in hover position and angle scoring functions for
        # when we are near perch. These can be added once the sim is upgraded

        # Max rotor speed [rad/s] from perch to crosswind and from crosswind to
        # perch.
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverAscend', 'kFlightModeHoverPayOut',
             'kFlightModeHoverPrepTransformGsUp',
             'kFlightModeHoverTransformGsUp', 'kFlightModeHoverFullLength',
             'kFlightModeHoverAccel'],
            rotors.MaxRotorSpeedsScoringFunction(215.0, 225.0, severity=2), 0.0,
            'Hover - Perch to CW'),
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverTransOut', 'kFlightModeHoverPrepTransformGsDown',
             'kFlightModeHoverTransformGsDown', 'kFlightModeHoverReelIn',
             'kFlightModeHoverDescend'],
            rotors.MaxRotorSpeedsScoringFunction(215.0, 225.0, severity=2), 0.0,
            'Hover - CW to Perch'),

        # Max rotors thrust saturation duration [s] from perch to crosswind and
        # from crosswind to perch.
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverAscend', 'kFlightModeHoverPayOut',
             'kFlightModeHoverPrepTransformGsUp',
             'kFlightModeHoverTransformGsUp', 'kFlightModeHoverFullLength'],
            rotors.ThrustMomentSaturationDurationScoringFunction(
                'thrust', 0.5, 1.0, severity=4),
            flight_mode_time_threshold=0.0,
            filter_name_prefix='Hover - Perch to CW'),
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverTransOut', 'kFlightModeHoverPrepTransformGsDown',
             'kFlightModeHoverTransformGsDown', 'kFlightModeHoverReelIn',
             'kFlightModeHoverDescend'],
            rotors.ThrustMomentSaturationDurationScoringFunction(
                'thrust', 0.5, 1.0, severity=4),
            flight_mode_time_threshold=0.0,
            filter_name_prefix='Hover - CW to Perch'),

        # Max rotors pitching moment saturation duration [s] from perch to
        # crosswind and from crosswind to perch.
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverAscend', 'kFlightModeHoverPayOut',
             'kFlightModeHoverPrepTransformGsUp',
             'kFlightModeHoverTransformGsUp', 'kFlightModeHoverFullLength'],
            rotors.ThrustMomentSaturationDurationScoringFunction(
                'moment_y', 0.5, 1.0, severity=4),
            flight_mode_time_threshold=0.0,
            filter_name_prefix='Hover - Perch to CW'),
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverTransOut', 'kFlightModeHoverPrepTransformGsDown',
             'kFlightModeHoverTransformGsDown', 'kFlightModeHoverReelIn',
             'kFlightModeHoverDescend'],
            rotors.ThrustMomentSaturationDurationScoringFunction(
                'moment_y', 0.5, 1.0, severity=4),
            flight_mode_time_threshold=0.0,
            filter_name_prefix='Hover - CW to Perch'),

        # Max rotors yawing moment saturation duration [s] from perch to
        # crosswind and from crosswind to perch.
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverAscend', 'kFlightModeHoverPayOut',
             'kFlightModeHoverPrepTransformGsUp',
             'kFlightModeHoverTransformGsUp', 'kFlightModeHoverFullLength'],
            rotors.ThrustMomentSaturationDurationScoringFunction(
                'moment_z', 0.5, 1.0, severity=4),
            flight_mode_time_threshold=0.0,
            filter_name_prefix='Hover - Perch to CW'),
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverTransOut', 'kFlightModeHoverPrepTransformGsDown',
             'kFlightModeHoverTransformGsDown', 'kFlightModeHoverReelIn',
             'kFlightModeHoverDescend'],
            rotors.ThrustMomentSaturationDurationScoringFunction(
                'moment_z', 0.5, 1.0, severity=4),
            flight_mode_time_threshold=0.0,
            filter_name_prefix='Hover - CW to Perch'),

        # The tether tension [kN] must be high enough during reel-in and
        # reel-out modes to prevent the tether from coming out of the
        # level-wind.
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverPayOut', 'kFlightModeHoverPrepTransformGsUp',
             'kFlightModeHoverTransformGsUp'], tether.TensionMinScoringFunction(
                 2.0, 1.0, severity=4), 0.0, 'Hover - Perch to Transform'),
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverTransformGsDown', 'kFlightModeHoverReelIn'],
            tether.TensionMinScoringFunction(
                2.0, 1.0, severity=4), 0.0, 'Hover - Transform to Perch'),

        # Tether Tension should follow the tension command.
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverPayOut', 'kFlightModeHoverPrepTransformGsUp',
             'kFlightModeHoverTransformGsUp', 'kFlightModeHoverFullLength',
             'kFlightModeHoverPrepTransformGsDown',
             'kFlightModeHoverTransformGsDown', 'kFlightModeHoverReelIn'],
            scoring_functions.PayoutFilter(
                hover.HoverTensionControlScoringFunction(
                    2.0, 5.0, severity=1), 7.0),
            flight_mode_time_threshold=0.0, filter_name_prefix=None),

        # Tether Tension should not oscillate. Limits [kN] and
        # cut-off frequency [Hz] are chosen from CW01-CW03 flight data.
        # See b/128534501.
        scoring_functions.PayoutFilter(
            scoring_functions.FlightModeFilter(
                ['kFlightModeHoverPayOut', 'kFlightModeHoverPrepTransformGsUp',
                 'kFlightModeHoverTransformGsUp', 'kFlightModeHoverFullLength'],
                tether.TetherTensionOscillationScoringFunction(
                    1.0, 1.2, severity=4, cut_off_freq=1.0/20.0), 0.0,
                'Hover - Perch to Accel'), 5.0),
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverTransOut'],
            tether.TetherTensionOscillationScoringFunction(
                3.0, 5.0, severity=3, cut_off_freq=1.0/5.0),
            0.0, 'HoverTransout'),
        scoring_functions.PayoutFilter(
            scoring_functions.FlightModeFilter(
                ['kFlightModeHoverPrepTransformGsDown',
                 'kFlightModeHoverTransformGsDown', 'kFlightModeHoverReelIn'],
                tether.TetherTensionOscillationScoringFunction(
                    1.0, 1.2, severity=4, cut_off_freq=1.0/20.0), 0.0,
                'Hover - PrepTransformDn to Perch'), 5.0),

        # Tether elevation should not oscillate. Limits [deg] and
        # cut-off frequency [Hz] are chosen from CW01 and CW02 flight data.
        # See b/119215536.
        scoring_functions.PayoutFilter(
            hover.TetherElevationOscillationScoringFunction(
                1.0, 2.0, severity=4, cut_off_freq=1.0/20.0),
            min_payout=6.0, flight_mode_filter='PayOut'),
        scoring_functions.PayoutFilter(
            hover.TetherElevationOscillationScoringFunction(
                1.0, 2.0, severity=4, cut_off_freq=1.0/20.0),
            min_payout=6.0, flight_mode_filter='ReelIn'),

        # Tether pitch in hover must remain small enough to ensure effective
        # bridle roll stiffening. The lower "good" limit was determined from
        # flight testing (see https://goo.gl/UMnMLd). The upper limit is driven
        # by bridle interference with the starboard pylon.
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverAscend', 'kFlightModeHoverPayOut',
             'kFlightModeHoverPrepTransformGsUp',
             'kFlightModeHoverTransformGsUp', 'kFlightModeHoverFullLength'],
            tether.SustainedPitchDegScoringFunction(-55.0, -50.0, 16.0, 17.0,
                                                    severity=4,
                                                    sustained_duration=1.0),
            filter_name_prefix='Hover - Perch to Accel'),
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverPrepTransformGsDown',
             'kFlightModeHoverTransformGsDown', 'kFlightModeHoverReelIn',
             'kFlightModeHoverDescend'],
            tether.SustainedPitchDegScoringFunction(-55.0, -50.0, 16.0, 17.0,
                                                    severity=4,
                                                    sustained_duration=1.0),
            filter_name_prefix='Hover - TransOut to Perch'),

        # The period [s] of the roll mode in hover must stay below an acceptable
        # threshold, indicating sufficient roll bridle stiffening. The allowable
        # threshold was determined following the High-Hover-01 flight test.
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverPayOut', 'kFlightModeHoverPrepTransformGsUp',
             'kFlightModeHoverTransformGsUp', 'kFlightModeHoverFullLength'],
            tether.RollPeriodScoringFunction(10.0, 12.0, severity=4),
            filter_name_prefix='Hover - Perch to CW'),
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverTransOut', 'kFlightModeHoverPrepTransformGsDown',
             'kFlightModeHoverTransformGsDown', 'kFlightModeHoverReelIn'],
            tether.RollPeriodScoringFunction(10.0, 12.0, severity=4),
            filter_name_prefix='Hover - CW to Perch'),

        scoring_functions.FlightModeFilter(
            'kFlightModeHoverDescend',
            tether.TensionMinScoringFunction(2.0, 1.0, severity=4)),

        scoring_functions.FlightModeFilter(
            'kFlightModeHoverAscend',
            tether.TensionMinScoringFunction(2.0, 1.0, severity=4)),
    )

    if flight_plan == 'TurnKey':
      self.scoring_functions += (

          ###########################################
          # HoverAccel and TransIn scoring functions.
          ###########################################
          # Must spend some minimum time [s] in the hoverAccel flight mode.
          scoring_functions.FlightModeFilter(
              'kFlightModeHoverAccel',
              scoring_functions.DurationScoringFunction(1.5, 3.0, 10.0, 15.0,
                                                        severity=1)),

          # Must spend some minimum time [s] in the TransIn flight mode.
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              scoring_functions.DurationScoringFunction(5.0, 6.0, 20.0, 25.0,
                                                        severity=1)),

          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              trans_in.TransInPitchForwardDurationScoringFunction(5.0, 3.0, 5.0,
                                                                  severity=1)),

          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              aero.MinAlphaDegScoringFunction(
                  -10.0, -8.0,
                  np.rad2deg(
                      limits['trans_in_pitched_forward_alpha']) - 1.0,
                  np.rad2deg(limits['trans_in_pitched_forward_alpha']),
                  severity=2)),

          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              trans_in.AfterPitchForwardScoringFunction(
                  np.rad2deg(limits['trans_in_pitched_forward_alpha']),
                  aero.AlphaDegScoringFunction(
                      *np.rad2deg(limits['crosswind_alpha_limits']),
                      severity=2))),

          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              trans_in.AfterPitchForwardScoringFunction(
                  np.rad2deg(limits['trans_in_pitched_forward_alpha']),
                  aero.BetaDegScoringFunction(
                      *np.rad2deg(limits['trans_in_beta_limits']),
                      severity=2))),

          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              trans_in.TransInFinalLateralVelocityErrorScoringFunction(
                  5.0, 10.0, severity=2)),

          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              trans_in.FinalAirspeedScoringFunction(25.0, 20.0, severity=2)),

          # TODO: This should be increased when trans-in tension
          # application is improved.
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              tether.TensionMaxScoringFunction(10.0, 5.0, severity=2)),

          # The simulated tether pitch has very brief transients (<0.5 s) where
          # it peaks outside of "good" limits. This is a simulator artifact
          # which is ignored by looking for occurrences of sustained tether
          # pitch outside of limits.
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              tether.SustainedPitchDegScoringFunction(-55.0, -50.0, 10.0, 17.0,
                                                      severity=3,
                                                      sustained_duration=0.5)),

          # Tether roll scoring function with high tension threshold, driven by
          # structural loads limits (see load cases RPX HEHR and RPX HELR in
          # go/makanienvelope).
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              tether.RollDegScoringFunction(-11.5, -9.5, 23.8, 25.8,
                                            severity=3,
                                            tension_threshold=200e3)),

          # Maximum motor speed [rad/s] should nominally be in the range
          # [200, 210] rad/s in trans-in. It should not exceed 220 rad/s.
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              rotors.MaxRotorSpeedsScoringFunction(210.0, 220.0, severity=2)),

          # Maximum rotor thrust [N] allowed in hover-accel and trans-in. The
          # good value corresponds to the maximum thrust that the controller
          # is allowed to command. The bad value is the peak thrust defining the
          # load case HA (see go/makani-gen3-rotor-loads).
          scoring_functions.FlightModeFilter(
              'kFlightModeHoverAccel',
              rotors.MaxRotorThrustScoringFunction(3800.0, 4400.0, severity=2)),
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              rotors.MaxRotorThrustScoringFunction(3800.0, 4400.0, severity=2)),

          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              trans_in.AfterPitchForwardScoringFunction(
                  np.rad2deg(limits['trans_in_pitched_forward_alpha']),
                  tether.PitchMomentCoefficientScoringFunction(
                      -2.0, -1.0, 0.1, 0.2, severity=1))),

          # Key control surface saturations are shown with a non-zero score if
          # the surface is saturated during trans-in.
          # TODO: Once we have implemented flight quality metrics these
          # limits should be determined based on the percent time [%] that each
          # control surface was saturated during trans-in in flight tests.
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              aero.SurfaceSaturationScoringFunction(['Ele'],
                                                    0.0, 100.0, severity=4)),
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              aero.SurfaceSaturationScoringFunction(['Rud'],
                                                    0.0, 100.0, severity=4)),

          # Max distance [m] wing overflies the GS into the wind during
          # trans-in. See b/117102168 for discussion of limits and severity.
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              kinematics.MaxWingOverflyGSScoringFunction(0.0, 45.0,
                                                         severity=4)),

          # Trans-in pitch rate scoring function.
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              kinematics.PitchRateScoringFunction(0.75, 1.25, severity=2)),

          # Maximum servo hinge moments [N-m] of elevator should be
          # less than 135 N-m to allow for redundant actuation, and should never
          # exceed 281.6 N-m which is the limit for paired actuation, as
          # documented in go/makani_servo_limits.
          scoring_functions.FlightModeFilter(
              'kFlightModeTransIn',
              loads.MaxElevatorServoMoment(135.0, 281.6, severity=2)),

          ###########################################
          # Crosswind scoring functions.
          ###########################################

          # Must spend some minimum time [s] in the crosswind flight mode.
          scoring_functions.CrosswindFilter(
              scoring_functions.DurationScoringFunction(30.0, 60.0,
                                                        10000.0, 15000.0,
                                                        severity=1),
              flight_mode_time_threshold=common_flight_mode_time_threshold),

          # Azimuth error scoring function. This scoring function checks that
          # the computed azimuth error in the GS02 high tension controller stays
          # within design limits. See https://goo.gl/L7W9Yw section 3.1.
          scoring_functions.CrosswindFilter(
              gs02.GsAzimuthErrorScoringFunction(
                  -23.0, -20.0, 20.0, 23.0, severity=3),
              flight_mode_time_threshold=common_flight_mode_time_threshold),

          # Angle-of-attack [deg] limits.
          # The MainWingAlphaScoringFunction evaluates the alpha [deg] of each
          # airfoil seen along the span of the main wing, as evaluated by the
          # Super Simple Aero Model (SSAM).
          # We include separate scoring functions for the entire crosswind
          # flight, including the initial transients, and the steady crosswind
          # flight.
          scoring_functions.CrosswindFilter(
              aero.MainWingAlphaScoringFunction(
                  *np.rad2deg(limits['crosswind_alpha_limits']),
                  severity=2,
                  airspeed_threshold=20.0),
              flight_mode_time_threshold=common_flight_mode_time_threshold),
          scoring_functions.CrosswindFilter(
              aero.MainWingAlphaScoringFunction(
                  *np.rad2deg(limits['crosswind_alpha_limits']),
                  severity=3,
                  airspeed_threshold=20.0,
                  steady_flight=True),
              flight_mode_time_threshold=steady_flight_mode_time),

          # Angle-of-sideslip [deg] limits.
          # We include separate scoring functions for the entire crosswind
          # flight, including the initial transients, and the steady crosswind
          # flight.
          scoring_functions.CrosswindFilter(
              aero.BetaDegScoringFunction(
                  *np.rad2deg(limits['crosswind_beta_limits']),
                  severity=1,
                  airspeed_threshold=20.0),
              flight_mode_time_threshold=common_flight_mode_time_threshold),
          scoring_functions.CrosswindFilter(
              aero.BetaDegScoringFunction(
                  *np.rad2deg(limits['crosswind_beta_limits']),
                  severity=3,
                  airspeed_threshold=20.0,
                  steady_flight=True),
              flight_mode_time_threshold=steady_flight_mode_time),

          # Root-mean-square error of alpha [deg], beta [deg], and airspeed
          # [m/s] during steady crosswind.
          scoring_functions.CrosswindFilter(
              aero.AlphaRmsScoringFunction(0.0, 2.0, severity=0),
              flight_mode_time_threshold=steady_flight_mode_time),

          scoring_functions.CrosswindFilter(
              aero.BetaRmsScoringFunction(0.0, 5.0, severity=0),
              flight_mode_time_threshold=steady_flight_mode_time),

          scoring_functions.CrosswindFilter(
              aero.AirspeedRmsScoringFunction(0.0, 1.0, severity=0),
              flight_mode_time_threshold=steady_flight_mode_time),

          # Aero angles error [deg] allowed during steady crosswind (applied
          # after the transient from transition-in has died). The limits of the
          # scoring functions are somewhat arbitrary and based on flight
          # testing.
          scoring_functions.CrosswindFilter(
              aero.AlphaDegErrorScoringFunction(2.0, 4.0, severity=3,
                                                steady_flight=True),
              flight_mode_time_threshold=steady_flight_mode_time),
          scoring_functions.CrosswindFilter(
              aero.BetaDegErrorScoringFunction(3.0, 6.0, severity=4,
                                               steady_flight=True),
              flight_mode_time_threshold=steady_flight_mode_time),

          # Min airspeed [m/s] allowed during steady crosswind. This
          # score should only be applied after the transient from
          # transition-in has died.
          scoring_functions.CrosswindFilter(
              aero.AirspeedMinScoringFunction(
                  *LimitsHelper(
                      [params['control']['crosswind']['power']['min_airspeed']],
                      [10.0, 5.0]),
                  severity=4),
              flight_mode_time_threshold=steady_flight_mode_time),

          # Airspeed error [m/s] allowed during steady crosswind. This
          # score should only be applied after the transient from
          # transition-in has died.
          scoring_functions.CrosswindFilter(
              crosswind.AirspeedErrorScoringFunction(5.0, 10.0, severity=2),
              flight_mode_time_threshold=steady_flight_mode_time),

          # Radius error [m] allowed during steady crosswind.  This
          # score should only be applied after the transient from
          # transition-in has died.
          scoring_functions.CrosswindFilter(
              crosswind.RadiusErrorScoringFunction(-50.0, -25.0, 50.0, 100.0,
                                                   severity=1),
              flight_mode_time_threshold=steady_flight_mode_time),

          # Low tension tether pitch scoring function, used to evaluate how
          # close the system is to an interference between the bridle release
          # mechanism and the wing (bending of the release starts at approx.
          # -30 to -40 deg, but is acceptable at low tensions up to -50 deg
          # tether pitch). The upper "good" limit is driven by bridle
          # interference with the starboard pylon (at tether pitch=17.7 deg).
          scoring_functions.CrosswindFilter(
              tether.PitchDegScoringFunction(-55.0, -50.0, 16.0, 17.0,
                                             severity=1,
                                             tension_threshold=1000.0),
              flight_mode_time_threshold=common_flight_mode_time_threshold),

          # High tension tether roll scoring function, used to evaluate how
          # close the controller is to exceeding structural load limits. Roll
          # limits are based around the RPX HEHR and RPX HELR cases in
          # go/makanienvelope, as it is impossible to fly with the tighter
          # limits in the original specification.
          scoring_functions.CrosswindFilter(
              tether.RollDegScoringFunction(-11.5, -9.5, 23.8, 25.8, severity=2,
                                            tension_threshold=200e3),
              flight_mode_time_threshold=common_flight_mode_time_threshold),

          # Minimum tension [kN] allowed during steady crosswind. The limits are
          # discussed in b/117490377.
          scoring_functions.CrosswindFilter(
              tether.TensionMinScoringFunction(25.0, 20.0, severity=3),
              flight_mode_time_threshold=steady_flight_mode_time),

          # Maximum motor speed [rad/s] should nominally be in the range
          # [220, 240] rad/s in steady crosswind flight. It should not
          # exceed 250 rad/s (see b/67742392).
          scoring_functions.CrosswindFilter(
              rotors.MaxRotorSpeedsScoringFunction(240.0, 250.0, severity=2),
              flight_mode_time_threshold=common_flight_mode_time_threshold),

          # Maximum motor torque [N-m] in steady crosswind flight should
          # peak below 850 N-m. It should not exceed 900 N-m.
          # see "Makani Gen2.3 key motor parameters with 850V.pdf" on Drive)
          scoring_functions.CrosswindFilter(
              rotors.MaxMotorTorqueScoringFunction(850.0, 900.0, severity=2),
              flight_mode_time_threshold=common_flight_mode_time_threshold),

          # Yb-acceleration range [m/s^2] allowed during entire crosswind. The
          # limits are based around the rpxHAHB and rpxHALB load cases in
          # go/makanienvelope.
          scoring_functions.CrosswindFilter(
              loads.YbAccelerationScoringFunction(-79.0, -42.0, 20.0, 25.0,
                                                  severity=0),
              flight_mode_time_threshold=common_flight_mode_time_threshold),

          # Key control surface saturations are shown with a non-zero score if
          # the surface is saturated during a particular loop of crosswind.
          # The SurfaceSaturationScoringFunction is written such that while in a
          # crosswind configuration, the maximum percentage time spent saturated
          # during any particular loop is what is returned as a score. The
          # choice of an upper limit of 2.5 percent is the equivalent of saying
          # the kite should not go open loop for more than a total of 9 degrees
          # within any single loop, i.e. 360 degrees*0.025 = 9 degrees
          scoring_functions.CrosswindFilter(
              aero.SurfaceSaturationScoringFunction(['A1'], 0.0, 2.5,
                                                    severity=4),
              flight_mode_time_threshold=common_flight_mode_time_threshold),
          scoring_functions.CrosswindFilter(
              aero.SurfaceSaturationScoringFunction(['A2'], 0.0, 2.5,
                                                    severity=4),
              flight_mode_time_threshold=common_flight_mode_time_threshold),
          scoring_functions.CrosswindFilter(
              aero.SurfaceSaturationScoringFunction(['A7'], 0.0, 2.5,
                                                    severity=4),
              flight_mode_time_threshold=common_flight_mode_time_threshold),
          scoring_functions.CrosswindFilter(
              aero.SurfaceSaturationScoringFunction(['A8'], 0.0, 2.5,
                                                    severity=4),
              flight_mode_time_threshold=common_flight_mode_time_threshold),
          scoring_functions.CrosswindFilter(
              aero.SurfaceSaturationScoringFunction(['Ele'], 0.0, 2.5,
                                                    severity=4),
              flight_mode_time_threshold=common_flight_mode_time_threshold),
          scoring_functions.CrosswindFilter(
              aero.SurfaceSaturationScoringFunction(['Rud'], 0.0, 2.5,
                                                    severity=4),
              flight_mode_time_threshold=common_flight_mode_time_threshold),

          # Check minimum reasonable altitude [m] in crosswind flight.
          scoring_functions.CrosswindFilter(
              kinematics.KiteHeightAglScoringFunctionWrapper(
                  kinematics.MinKiteHeightAglScoringFunction(
                      *LimitsHelper(
                          [limits['min_altitude']],
                          [-10.0, -25.0]),
                      severity=3))),

          # Average generated power [kW] in crosswind.
          scoring_functions.CrosswindFilter(
              power.PowerGeneratedMeanScoringFunction(600.0, 0.0, severity=1)),

          ###########################################
          # CrosswindPrepTransOut scoring functions.
          ###########################################

          # Must spend some minimum time [s] in the prep-trans-out flight mode.
          scoring_functions.FlightModeFilter(
              'kFlightModeCrosswindPrepTransOut',
              scoring_functions.DurationScoringFunction(1.0, 3.0, 180.0, 300.0,
                                                        severity=1)),

          # The kite must not deviate too far from the tether sphere during
          # CrosswindPrepTransOut. The limits [m] are selected based on flight
          # test experience (see b/147883975).
          scoring_functions.FlightModeFilter(
              'kFlightModeCrosswindPrepTransOut',
              kinematics.TetherSphereDeviationScoringFunction(
                  2.0, 4.0, severity=4, mean_tether_tension=100e3)),

          # The max TransOut airspeed command immediately before and after
          # the flare is 40 m/s. We restrict the scoring of Alpha and Beta in
          # CrosswindPrepTransOut to airspeeds above this threshold. All limits
          # are the same as in CrosswindNormal except the good limits for beta
          # are changed to -10, 10 degrees based on flight test results.
          # See b/77638038 for details.
          scoring_functions.FlightModeFilter(
              'kFlightModeCrosswindPrepTransOut',
              aero.MainWingAlphaScoringFunction(
                  *np.rad2deg(limits['crosswind_alpha_limits']),
                  severity=2,
                  airspeed_threshold=40.0)),
          scoring_functions.FlightModeFilter(
              'kFlightModeCrosswindPrepTransOut',
              aero.BetaDegScoringFunction(
                  *np.rad2deg(limits['prep_trans_out_beta_limits']),
                  severity=1,
                  airspeed_threshold=40.0)),

          # Radius error [m] allowed during prep-trans-out.
          scoring_functions.FlightModeFilter(
              'kFlightModeCrosswindPrepTransOut',
              crosswind.RadiusErrorScoringFunction(-50.0, -25.0, 50.0, 100.0,
                                                   severity=2)),

          # Maximum servo hinge moments [N-m] of elevator should be
          # less than 135 N-m to allow for redundant actuation, and should never
          # exceed 281.6 N-m which is the limit for paired actuation, as
          # documented in go/makani_servo_limits.
          scoring_functions.FlightModeFilter(
              'kFlightModeCrosswindPrepTransOut',
              loads.MaxElevatorServoMoment(135.0, 281.6, severity=4)),

          # Rudder surface saturations are shown with a non-zero score if
          # the surface is saturated during CrosswindPrepTransOut.
          # TODO: Once we have implemented flight quality metrics these
          # limits should be determined based on the percent time [%] that each
          # control surface was saturated during crosswind in flight tests.
          scoring_functions.FlightModeFilter(
              'kFlightModeCrosswindPrepTransOut',
              aero.SurfaceSaturationScoringFunction(['Rud'], 0.0, 100.0,
                                                    severity=2)),

          # Maximum rotor thrust [N] allowed in trans-out. The limits are equal
          # to the ones set in hover-accel and trans-in.
          scoring_functions.FlightModeFilter(
              'kFlightModeHoverTransOut',
              rotors.MaxRotorThrustScoringFunction(3800.0, 4400.0, severity=2)),

          ###########################################
          # HoverTransOut scoring functions.
          ###########################################

          # Must spend some minimum time [s] in the Hover-trans-out flight mode.
          scoring_functions.FlightModeFilter(
              'kFlightModeHoverTransOut',
              scoring_functions.DurationScoringFunction(1.0, 5.0,
                                                        10000.0, 15000.0,
                                                        severity=1)),

          # The kite must not deviate too far from the tether sphere during
          # TransOut. The limits [m] are selected based on flight test
          # experience (see b/147883975).
          scoring_functions.FlightModeFilter(
              'kFlightModeHoverTransOut',
              kinematics.TetherSphereDeviationScoringFunction(
                  5.0, 8.0, severity=4, mean_tether_tension=100e3)),

          # Based on experience from flight testing (see go/makani-ecr-480), the
          # kite must trans-out within a safe range of altitudes above ground
          # level [m] in order to prevent very high hover (upper limit) and a
          # sideway trans-out maneuver (lower limit). The lower limit depends on
          # the height of the last loop in CrosswindPrepTransOut.
          scoring_functions.FlightModeFilter(
              'kFlightModeHoverTransOut',
              kinematics.KiteHeightAglScoringFunctionWrapper(
                  kinematics.KiteHeightAglScoringFunction(
                      100.0, 120.0, 190.0, 220.0, severity=3))),

          # The maximum pitch rate should not exceed 1.25 rad/s because
          # of gyroscopic loads on the pylons.
          scoring_functions.FlightModeFilter(
              'kFlightModeHoverTransOut',
              kinematics.PitchRateScoringFunction(0.75, 1.25, severity=1)),

          # Sustained tether pitch scoring function.
          scoring_functions.FlightModeFilter(
              'kFlightModeHoverTransOut',
              tether.SustainedPitchDegScoringFunction(-55.0, -50.0, 10.0, 17.0,
                                                      severity=1,
                                                      sustained_duration=3.0)),

          # Maximum servo hinge moments [N-m] of elevator should be
          # less than 135 N-m to allow for redundant actuation, and should never
          # exceed 281.6 N-m which is the limit for paired actuation, as
          # documented in go/makani_servo_limits.
          scoring_functions.FlightModeFilter(
              'kFlightModeHoverTransOut',
              loads.MaxElevatorServoMoment(135.0, 281.6, severity=0)),

          ###########################################
          # HoverDescend scoring functions.
          ###########################################

          # Duration [s] of HoverDescend flight mode. This scoring function is
          # used to verify that the kite has progressed to this flight mode. The
          # upper good limit is set intentionally high because the flight mode
          # will run as long as the simulation does.
          scoring_functions.FlightModeFilter(
              'kFlightModeHoverDescend',
              scoring_functions.DurationScoringFunction(
                  -1.0, 0.0, 500.0, 1000.0, severity=1,
                  extra_system_labels=['experimental'])),
      )

    ###########################################
    # GS02-specific scoring functions.
    ###########################################

    self.scoring_functions += (
        scoring_functions.FlightModeFilter(
            'kFlightModeHoverPrepTransformGsUp',
            scoring_functions.DurationScoringFunction(0.0, 20.0, 45.0, 300.0,
                                                      severity=1)),

        scoring_functions.FlightModeFilter(
            'kFlightModeHoverPrepTransformGsDown',
            scoring_functions.DurationScoringFunction(0.0, 20.0, 45.0, 300.0,
                                                      severity=1)),

        # The tether azimuth must be within a certain range during descend
        # to track the perch panel. The nominal azimuth window in platform
        # frame is [84.55, 92.47] deg according to b/137225712.
        scoring_functions.FlightModeFilter(
            'kFlightModeHoverAscend',
            hover.PanelAzimuthTrackingScoringFunction(
                84.55, 86.55, 90.47, 92.47, severity=4)),

        scoring_functions.FlightModeFilter(
            'kFlightModeHoverDescend',
            hover.PanelAzimuthTrackingScoringFunction(
                84.55, 86.55, 90.47, 92.47, severity=4)),

        # Check whether the kite is being hit by the panel by examining the
        # vertical acceleration.
        # See b/137227472.
        scoring_functions.FlightModeFilter(
            'kFlightModeHoverAscend',
            loads.ZgAccelerationScoringFunction(
                -10.0, -15.0, severity=4)),

        scoring_functions.FlightModeFilter(
            'kFlightModeHoverDescend',
            loads.ZgAccelerationScoringFunction(
                -10.0, -15.0, severity=4)),
    )

    self.scoring_functions += (
        # There is no explicit requirement on the size of detwist command jumps,
        # but if the command scheme is implemented sensibly, then command jumps
        # across flight modes should not be too large.
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverFullLength', 'kFlightModeHoverAccel',
             'kFlightModeTransIn', 'kFlightModeCrosswindNormal',
             'kFlightModeCrosswindPrepTransOut', 'kFlightModeHoverTransOut',
             'kFlightModeHoverPrepTransformGsDown'],
            gs02.GsDetwistCommandJumpScoringFunction(45.0, 90.0, severity=1),
            filter_name_prefix='HighTension'),
    )

    self.scoring_functions += (
        # Limit on detwist command rate [deg/s]. Limit is 1.257 rad/s
        # (72 deg/s).
        scoring_functions.FlightModeFilter(
            ['kFlightModeHoverFullLength', 'kFlightModeHoverAccel',
             'kFlightModeTransIn', 'kFlightModeCrosswindNormal',
             'kFlightModeCrosswindPrepTransOut', 'kFlightModeHoverTransOut',
             'kFlightModeHoverPrepTransformGsDown'],
            gs02.GsDetwistCommandRateScoringFunction(60.0, 72.0, severity=1),
            filter_name_prefix='HighTension'),
    )

    self.scoring_functions += (
        # Main to side lobe ratio in dB. Current limits of -10 and -6 dB are
        # based on observation from prior flights and simulation.
        scoring_functions.FlightModeFilter(
            ['kFlightModeTransIn', 'kFlightModeCrosswindNormal',
             'kFlightModeCrosswindPrepTransOut', 'kFlightModeHoverTransOut'],
            gs02.GsDetwistOscillationsScoringFunction(-10.0, -6.0, severity=3),
            filter_name_prefix='Crosswind - '),
    )

    ###########################################
    # Software performance scoring functions.
    ###########################################

    self.scoring_functions += (
        # Maximum absolute difference [s] between the sampling time parameter
        # and the first difference of the time telemetry.
        software.MaxSampleTimeErrorScoringFunction(0.0, 1e-6, severity=1),
    )

    ###########################################
    # Offshore scoring functions.
    ###########################################
    if offshore:
      self.scoring_functions += (
          # Distance from water line on the buoy to a threshold. The threshold
          # is the location of the top deck, as specified by the variable
          # 'top_deck_pos_z_v' in config/m600/buoy.py, measured from the origin
          # of the vessel frame (positive downwards).
          buoy.BuoyWaterLineScoringFunction(6.3, 5.3, severity=3),
      )

      self.scoring_functions += (
          # Peak yaw angle of the buoy wrt the rest (equilibrium) azimuth, as
          # defined by the mooring line systemin the file
          # config/m600/sim/buoy_sim.py.
          buoy.BuoyYawAngleScoringFunction(60.0, 90.0, severity=3),
      )

      self.scoring_functions += (
          # Peak acceleration of the vessel frame origin, measured in gs.
          buoy.BuoyVesselOriginAccelScoringFunction(1.0, 2.0, severity=2),
      )

    ###########################################
    # Other Oktoberkite scores.
    ###########################################
    if wing_model == 'oktoberkite':

      # Duration in each flight mode
      flight_modes = [
          'kFlightModeHoverAscend',
          'kFlightModeHoverPayOut',
          'kFlightModeHoverPrepTransformGsUp',
          'kFlightModeHoverTransformGsUp',
          'kFlightModeHoverFullLength',
          'kFlightModeHoverAccel',
          'kFlightModeTransIn',
          'kFlightModeCrosswindNormal',
          'kFlightModeCrosswindPrepTransOut',
          'kFlightModeHoverTransOut',
          'kFlightModeHoverPrepTransformGsDown',
          'kFlightModeHoverTransformGsDown',
          'kFlightModeHoverReelIn',
          'kFlightModeHoverDescend',
      ]
      for ii, flight_mode in enumerate(flight_modes):
        if flight_mode == 'kFlightModeCrosswindNormal':
          good_lower = 500.0
        else:
          good_lower = 250.0
        self.scoring_functions += (
            scoring_functions.FlightModeFilter(
                [flight_modes[ii]],
                scoring_functions.DurationScoringFunction(
                    0.1, good_lower, 10000.0, 15000.0,
                    severity=1, extra_system_labels='experimental'),
                filter_name_prefix=flight_mode.lstrip('kFlightMode')+' Basic'),
        )

    # Make sure there is no conflicting names.
    assert len(set(sf.GetName() for sf in self.scoring_functions)) == len(
        self.scoring_functions), (
            'One or more scoring function names are not unique.')
