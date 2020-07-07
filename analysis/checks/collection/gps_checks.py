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

"""Checklist items for GPS."""

from makani.analysis.checks import base_check
from makani.analysis.checks import check_range
from makani.avionics.common import novatel_types
from makani.avionics.common import septentrio_types
from makani.lib.python import c_helpers
import numpy

NOVATEL_SOLUTION_STATUS_HELPER = c_helpers.EnumHelper(
    'NovAtelSolutionStatus', novatel_types)
SEPTENTRIO_ERROR_HELPER = c_helpers.EnumHelper(
    'SeptentrioPvtError', septentrio_types)
# These are the aio_nodes for NovAtel and Septentrio
NOVATEL_SOURCES = ['FcA', 'FcB']
SEPTENTRIO_SOURCES = []
# These are the upper bounds of the normal and warning ranges
# for pos_sigma and vel_sigma. If a value is higher than the upper bound
# of the warning range, then we will throw an error.
POS_SIGMA_NORMAL_UPPER = 0.2
POS_SIGMA_WARNING_UPPER = 0.5
VEL_SIGMA_NORMAL_UPPER = 0.1
VEL_SIGMA_WARNING_UPPER = 0.3


class BaseNovAtelCheck(base_check.BaseCheckItem):
  """Base class for NovAtel checks."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source,
               normal_ranges=check_range.AllInclusiveRange(),
               warning_ranges=check_range.AllInclusiveRange()):
    self._source = source
    super(BaseNovAtelCheck, self).__init__(for_log, normal_ranges,
                                           warning_ranges)


class BaseSeptentrioCheck(base_check.BaseCheckItem):
  """Base class for Septentrio checks."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source,
               normal_ranges=check_range.AllInclusiveRange(),
               warning_ranges=check_range.AllInclusiveRange()):
    self._source = source
    super(BaseSeptentrioCheck, self).__init__(for_log, normal_ranges,
                                              warning_ranges)


class NovAtelSolutionTypeCheck(BaseNovAtelCheck):
  """Class to check NovAtel solution type."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source):
    super(NovAtelSolutionTypeCheck, self).__init__(for_log, source)

  def _RegisterInputs(self):
    return [
        self._Arg('NovAtelSolution', self._source, 'best_xyz.pos_sol_status')
    ]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, solution_type):
    """Checks that the solution type is 'SolComputed' else raises warning."""
    zero_range = check_range.Singleton(0)
    for error_name, error_value in NOVATEL_SOLUTION_STATUS_HELPER:
      # Skip 'SolComputed' because that is what we want the status to be.
      if error_value == 0:
        continue
      # Raise a warning if the status is equal to the error_value.
      self._CheckForFailure(self._source + ' ' + error_name,
                            numpy.array([int(s == error_value) for s in
                                         solution_type]),
                            zero_range, False)


class NovAtelDiffCheck(BaseNovAtelCheck):
  """Class to check the NovAtel diff_age."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source):
    normal_ranges = check_range.Interval((None, 3))
    warning_ranges = check_range.AllInclusiveRange()
    super(NovAtelDiffCheck, self).__init__(for_log, source, normal_ranges,
                                           warning_ranges)

  def _RegisterInputs(self):
    return [
        self._Arg('NovAtelSolution', self._source, 'best_xyz.diff_age')
    ]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, diff_age):
    self._CheckByRange(self._source + ' NovAtel diff_age too high', diff_age,
                       self._normal_ranges, self._warning_ranges)


class NovAtelPosSigmaCheck(BaseNovAtelCheck):
  """Class to check position sigma for NovAtel GPS."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source):
    normal_ranges = check_range.Interval((0, POS_SIGMA_NORMAL_UPPER))
    warning_ranges = check_range.Interval((0, POS_SIGMA_WARNING_UPPER))

    super(NovAtelPosSigmaCheck, self).__init__(for_log, source, normal_ranges,
                                               warning_ranges)

  def _RegisterInputs(self):
    """Register data to be used for the sigmas check."""
    return [
        self._Arg('NovAtelSolution', self._source, 'best_xyz.pos_x_sigma'),
        self._Arg('NovAtelSolution', self._source, 'best_xyz.pos_y_sigma'),
        self._Arg('NovAtelSolution', self._source, 'best_xyz.pos_z_sigma')
    ]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, x, y, z):
    if self._for_log:
      length = len(x)
      pos_sigma = numpy.array([numpy.linalg.norm([x[i], y[i], z[i]])
                               for i in xrange(length)])
    else:
      pos_sigma = numpy.linalg.norm([x, y, z])
    self._CheckByRange(self._source + ' NovAtel pos_sigma out of range',
                       pos_sigma, self._normal_ranges, self._warning_ranges)


class NovAtelVelSigmaCheck(BaseNovAtelCheck):
  """Class to check velocity sigma for NovAtel GPS."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source):
    normal_ranges = check_range.Interval((0, VEL_SIGMA_NORMAL_UPPER))
    warning_ranges = check_range.Interval((0, VEL_SIGMA_WARNING_UPPER))

    super(NovAtelVelSigmaCheck, self).__init__(for_log, source, normal_ranges,
                                               warning_ranges)

  def _RegisterInputs(self):
    """Register data to be used for the sigmas check."""
    return [
        self._Arg('NovAtelSolution', self._source, 'best_xyz.vel_x_sigma'),
        self._Arg('NovAtelSolution', self._source, 'best_xyz.vel_y_sigma'),
        self._Arg('NovAtelSolution', self._source, 'best_xyz.vel_z_sigma')
    ]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, x, y, z):
    if self._for_log:
      length = len(x)
      vel_sigma = numpy.array([numpy.linalg.norm([x[i], y[i], z[i]])
                               for i in xrange(length)])
    else:
      vel_sigma = numpy.linalg.norm([x, y, z])
    self._CheckByRange(self._source + ' NovAtel vel_sigma out of range',
                       vel_sigma, self._normal_ranges, self._warning_ranges)


class SeptentrioPvtCartesianErrorCheck(BaseSeptentrioCheck):
  """Class to check Septentrio pvt_cartesian.error."""

  @base_check.RegisterSpecs
  def __init(self, for_log, source):
    super(SeptentrioPvtCartesianErrorCheck, self).__init__(for_log, source)

  def _RegisterInputs(self):
    return [
        self._Arg('SeptentrioSolution', self._source, 'pvt_cartesian.error')
    ]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, status):
    """Checks that the solution type is 'SolComputed' else raises warning."""
    zero_range = check_range.Singleton(0)
    # Check all possible SeptentrioPvtError values.
    for error_name, error_value in SEPTENTRIO_ERROR_HELPER:
      # Skip the 'None' error.
      if error_value == 0:
        continue
      # Throw a warning if the status is equal to the error_value
      self._CheckForFailure(self._source + ' ' + error_name,
                            numpy.array([int(s == error_value)
                                         for s in status]),
                            zero_range, False)


class SeptentrioMeanCorrAgeCheck(BaseSeptentrioCheck):
  """Class to check Septentrio pvt_cartesian._mean_corr_age."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source):
    normal_ranges = check_range.Interval((None, 300))
    warning_ranges = check_range.AllExclusiveRange()
    super(SeptentrioMeanCorrAgeCheck, self).__init__(for_log, source,
                                                     normal_ranges,
                                                     warning_ranges)

  def _RegisterInputs(self):
    return [
        self._Arg('SeptentrioSolution', self._source,
                  'pvt_cartesian.mean_corr_age')
    ]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, mean_corr_age):
    self._CheckByRange(self._source + ' Septentrio mean_corr_age too high',
                       mean_corr_age, self._normal_ranges,
                       self._warning_ranges)


class SeptentrioPosSigmaCheck(BaseSeptentrioCheck):
  """Class to check pos_sigma for Septentrio GPS."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source):
    normal_ranges = check_range.Interval((0, POS_SIGMA_NORMAL_UPPER))
    warning_ranges = check_range.Interval((0, POS_SIGMA_WARNING_UPPER))
    super(SeptentrioPosSigmaCheck, self).__init__(for_log, source,
                                                  normal_ranges,
                                                  warning_ranges)

  def _RegisterInputs(self):
    """Register data to be used for the sigmas check."""
    return [
        self._Arg('SeptentrioSolution', self._source,
                  'pos_cov_cartesian.cov_xx'),
        self._Arg('SeptentrioSolution', self._source,
                  'pos_cov_cartesian.cov_yy'),
        self._Arg('SeptentrioSolution', self._source,
                  'pos_cov_cartesian.cov_zz'),
    ]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, x, y, z):
    if self._for_log:
      length = len(x)
      pos_sigma = numpy.array([numpy.sqrt(numpy.sum([x[i], y[i], z[i]]))
                               for i in xrange(length)])
    else:
      pos_sigma = numpy.linalg.norm([x, y, z])

    self._CheckByRange(self._source + ' Septentrio pos_sigma out of range',
                       pos_sigma, self._normal_ranges, self._warning_ranges)


class SeptentrioVelSigmaCheck(BaseSeptentrioCheck):
  """Class to check pos_sigma for Septentrio GPS."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source):
    normal_ranges = check_range.Interval((0, VEL_SIGMA_NORMAL_UPPER))
    warning_ranges = check_range.Interval((0, VEL_SIGMA_WARNING_UPPER))
    super(SeptentrioVelSigmaCheck, self).__init__(for_log, source,
                                                  normal_ranges,
                                                  warning_ranges)

  def _RegisterInputs(self):
    """Register data to be used for the sigmas check."""
    return [
        self._Arg('SeptentrioSolution', self._source,
                  'vel_cov_cartesian.cov_xx'),
        self._Arg('SeptentrioSolution', self._source,
                  'vel_cov_cartesian.cov_yy'),
        self._Arg('SeptentrioSolution', self._source,
                  'vel_cov_cartesian.cov_zz'),
    ]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, x, y, z):
    if self._for_log:
      length = len(x)
      vel_sigma = numpy.array([numpy.sqrt(numpy.sum([x[i], y[i], z[i]]))
                               for i in xrange(length)])
    else:
      vel_sigma = numpy.linalg.norm([x, y, z])

    self._CheckByRange(self._source + ' Septentrio vel_sigma out of range',
                       vel_sigma, self._normal_ranges, self._warning_ranges)


class AgreementCheck(base_check.BaseCheckItem):
  """Class to check whether the two GPSes agree."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, gps_type_per_source,
               normal_ranges=check_range.AllInclusiveRange(),
               warning_ranges=check_range.AllInclusiveRange()):
    assert 'FcA' in gps_type_per_source and 'FcB' in gps_type_per_source
    self._gps_type_per_source = gps_type_per_source
    super(AgreementCheck, self).__init__(for_log, normal_ranges, warning_ranges)

  def _RegisterInputs(self):
    """Register data to be used for the sigmas check."""
    args = []
    for source in ['FcA', 'FcB']:
      gps_type = self._gps_type_per_source[source]
      if gps_type == 'Septentrio':
        args += [
            self._Arg('SeptentrioSolution', source, 'pvt_cartesian.x'),
            self._Arg('SeptentrioSolution', source, 'pvt_cartesian.y'),
            self._Arg('SeptentrioSolution', source, 'pvt_cartesian.z'),
            self._Arg('SeptentrioSolution', source, 'pvt_cartesian.mode'),
            self._Arg('SeptentrioSolution', source,
                      'pvt_cartesian.timestamp.tow'),
        ]
      elif gps_type == 'NovAtel':
        args += [
            self._Arg('NovAtelSolution', source, 'best_xyz.pos_x'),
            self._Arg('NovAtelSolution', source, 'best_xyz.pos_y'),
            self._Arg('NovAtelSolution', source, 'best_xyz.pos_z'),
            self._Arg('NovAtelSolution', source, 'best_xyz.pos_type'),
            self._Arg('NovAtelSolution', source, 'best_xyz.timestamp.tow'),
        ]
      else:
        assert False
    return args

  def _GetGpsModeAndValidity(self, gps_mode, gps_type):
    if gps_type == 'Septentrio':
      gps_mode &= septentrio_types.kSeptentrioPvtModeBitSolutionMask
      gps_valid = (
          gps_mode == septentrio_types.kSeptentrioPvtModeRtkFixed)
    elif gps_type == 'NovAtel':
      gps_valid = (
          (gps_mode == novatel_types.kNovAtelSolutionTypeL1Int) |
          (gps_mode == novatel_types.kNovAtelSolutionTypeNarrowInt) |
          (gps_mode == novatel_types.kNovAtelSolutionTypeWideInt))
    else:
      assert False
    return gps_mode, gps_valid

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, fca_gps_x, fca_gps_y, fca_gps_z, fca_gps_mode,
             fca_gps_tow, fcb_gps_x, fcb_gps_y, fcb_gps_z, fcb_gps_mode,
             fcb_gps_tow):

    fca_gps_mode, fca_gps_valid = self._GetGpsModeAndValidity(
        fca_gps_mode, self._gps_type_per_source['FcA'])

    fcb_gps_mode, fcb_gps_valid = self._GetGpsModeAndValidity(
        fcb_gps_mode, self._gps_type_per_source['FcB'])

    if (not self._for_log) and not (fca_gps_valid and fcb_gps_valid):
      return

    if self._for_log:
      # We are using tow to find same-time updates on two GPSes.
      # TODO: This needs to be timestamp (combining tow and week) when
      # we start to have logs that span for more than 7 days.
      fcb_gps_tow, fcb_gps_unique_index = numpy.unique(
          fcb_gps_tow, return_index=True)
      fca_gps_tow, fca_gps_unique_index = numpy.unique(
          fca_gps_tow, return_index=True)

      fcb_gps_overlap_mask = numpy.in1d(fcb_gps_tow, fca_gps_tow)
      fcb_gps_x = fcb_gps_x[fcb_gps_unique_index][fcb_gps_overlap_mask]
      fcb_gps_y = fcb_gps_y[fcb_gps_unique_index][fcb_gps_overlap_mask]
      fcb_gps_z = fcb_gps_z[fcb_gps_unique_index][fcb_gps_overlap_mask]

      fca_gps_overlap_mask = numpy.in1d(fca_gps_tow, fcb_gps_tow)
      fca_gps_x = fca_gps_x[fca_gps_unique_index][fca_gps_overlap_mask]
      fca_gps_y = fca_gps_y[fca_gps_unique_index][fca_gps_overlap_mask]
      fca_gps_z = fca_gps_z[fca_gps_unique_index][fca_gps_overlap_mask]
      assert fcb_gps_x.size == fca_gps_x.size
      if not fcb_gps_x.size:
        return

    diff = numpy.sqrt(
        (fcb_gps_x - fca_gps_x) ** 2 + (fcb_gps_y - fca_gps_y) ** 2 +
        (fcb_gps_z - fca_gps_z) ** 2)

    self._CheckByRange(
        'GPS (%s) does not agree with GPS (%s)' % ('FcA', 'FcB'),
        diff, self._normal_ranges, self._warning_ranges)


class BaseCn0Checker(base_check.BaseCheckItem):
  """The monitor to check GPS carrier-to-noise ratio."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, source, cn0_field, num_field,
               type_field, name):
    """Initialize the voltage checker for a given servo.

    Args:
      for_log: True if this check is performed over a log. False if it is for
          realtime AIO messages.
      message_type: Type of the message.
      source: The AIO node that sent the message.
      cn0_field: Path to the C/N0 field.
      num_field: Path to the num_obs field.
      type_field: The type to extract particular signals.
      name: Name of the check item.
    """
    self._message_type = message_type
    self._source = source
    self._cn0_field = cn0_field
    self._num_field = num_field
    self._type_field = type_field
    super(BaseCn0Checker, self).__init__(for_log, name=name)

  def Cn0Field(self):
    return self._cn0_field

  def NumField(self):
    return self._num_field

  def TypeField(self):
    return self._type_field

  def _GetValidCn0(self, cn0, type_bits):
    """Select and compute valid C/N0 values.

    Args:
      cn0: A NumPy array of raw Cn0 values.
      type_bits: A NumPy array of signal types.

    Returns:
      A NumPy array of valid Cn0 values.
    """
    raise NotImplementedError

  def _RegisterInputs(self):
    """Register what fields will be used to calculate the check results."""
    data = []
    data.append(self._Arg(
        self._message_type, self._source, self._cn0_field))
    data.append(self._Arg(
        self._message_type, self._source, self._num_field))
    data.append(self._Arg(
        self._message_type, self._source, self._type_field))
    return data

  def GetAvgAndMaxCn0FromTimeSeries(self, cn0, num_obs, type_bits):
    avg_cn0s = []
    max_cn0s = []
    for n in range(cn0.shape[0]):
      avg_cn0, max_cn0, _ = self.GetAvgAndMaxCn0(
          cn0[n], num_obs[n], type_bits[n])
      avg_cn0s.append(avg_cn0)
      max_cn0s.append(max_cn0)
    return numpy.array(avg_cn0s), numpy.array(max_cn0s)

  def GetAvgAndMaxCn0(self, cn0, num_obs, type_bits):
    if num_obs == 0:
      return float('nan'), float('nan'), 0
    cn0 = numpy.array(cn0[:num_obs])
    type_bits = numpy.array(type_bits[:num_obs])
    cn0 = self._GetValidCn0(cn0, type_bits)
    if cn0.size:
      return numpy.average(cn0), numpy.max(cn0), len(cn0)
    else:
      return float('nan'), float('nan'), 0

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, cn0, num_obs, type_bits):
    """Check the carrier-to-noise ratio."""

    if num_obs == 0:
      return

    if self._for_log:
      avg_cn0, max_cn0 = self.GetAvgAndMaxCn0FromTimeSeries(
          cn0, num_obs, type_bits)
    else:
      avg_cn0, max_cn0, _ = self.GetAvgAndMaxCn0(cn0, num_obs, type_bits)

    avg_ranges = check_range.Interval([40.0, None])
    max_ranges = check_range.Interval([45.0, None])
    all_inclusive = check_range.AllInclusiveRange()
    self._CheckByRange('%s (Avg)' % self._name, avg_cn0, avg_ranges,
                       all_inclusive)
    self._CheckByRange('%s (Max)' % self._name, max_cn0, max_ranges,
                       all_inclusive)


class NovAtelCn0Checker(BaseCn0Checker):
  """Cn0 check for NovAtel receivers."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source, name):
    # See NovAtel OEM6 documentation for RANGE log. Table 126: Channel
    # Tracking Status on page 604 (rev 8) documents the meaning of the
    # status bits.
    super(NovAtelCn0Checker, self).__init__(
        for_log, 'NovAtelObservations', source, 'range.cn0', 'range.num_obs',
        'range.status_bits', name)

  def _GetValidCn0(self, cn0, type_bits):
    return cn0[(type_bits & 0x03E70000) == 0]


class SeptentrioCn0Checker(BaseCn0Checker):
  """Cn0 check for Septentrio receivers."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, source, name):
    # See Septentrio's SBF Reference Guide. Table 2.10: Signal Type.
    super(SeptentrioCn0Checker, self).__init__(
        for_log, 'SeptentrioObservations', source, 'meas_epoch.cn0',
        'meas_epoch.num_obs', 'meas_epoch.type', name)

  def _GetValidCn0(self, cn0, type_bits):
    # See page 18 of AsteRx-m Firmware v3.3.0 SBF Reference Guide.
    assert type_bits.itemsize == 8
    # Check bits 0-4 to select only L1 C/A signals.
    # Need to convert to float to avoid numpy's float->int casting restrictions.
    valid_cn0 = cn0[(type_bits & 0x1F) == 0].astype(float)
    valid_cn0 *= 0.25
    valid_cn0 += 10.0
    return valid_cn0


class GpsChecks(base_check.ListOfChecks):
  """The GPS checklist."""

  def __init__(self, for_log):
    self._items_to_check = [
        AgreementCheck(
            for_log, {'FcA': 'NovAtel', 'FcB': 'NovAtel'}, [[0.0, 1.0]]),
    ]

    for source in NOVATEL_SOURCES:
      self._items_to_check += [
          NovAtelSolutionTypeCheck(for_log, source),
          NovAtelDiffCheck(for_log, source),
          NovAtelPosSigmaCheck(for_log, source),
          NovAtelVelSigmaCheck(for_log, source),
      ]

    for source in SEPTENTRIO_SOURCES:
      self._items_to_check += [
          SeptentrioPvtCartesianErrorCheck(for_log, source),
          SeptentrioMeanCorrAgeCheck(for_log, source),
          SeptentrioPosSigmaCheck(for_log, source),
          SeptentrioVelSigmaCheck(for_log, source),
      ]
