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

"""Provides a Python interface to C-Sim databases.

This code was converted from:
https://codesearch.corp.google.com/makani/analysis/aero/load_database.py?dr=C&g=0&l=1

Note that the angles are all given in degrees instead of the standard C-Sim unit
of rad. This is because the databases are currently written in terms of degrees.
"""

from __future__ import absolute_import
import json
import numpy as np
from scipy.interpolate import RectBivariateSpline as rbsf


def UseRBSF(alphas, betas, data, alphad_order=2, betad_order=2):
  return rbsf(alphas, betas, data, kx=alphad_order, ky=betad_order, s=0)


class CSimDatabase(object):
  """Loads an aero database from a .json file."""

  def __init__(self, filename):
    with open(filename) as f:
      database = json.load(f)

    # Used for the sizing and interpolation purposes later on.
    self._num_alphads = database['num_alphas']
    self._num_betads = database['num_betas']
    self._alpha_limits = {
        'alpha_max': np.max(np.asarray(database['alphads'][0])),
        'alpha_min': np.min(np.asarray(database['alphads'][0]))
        }
    self._beta_limits = {
        'beta_max': np.max(np.asarray(database['betads'][0])),
        'beta_min': np.min(np.asarray(database['betads'][0]))
        }

    # alpha and beta must comprise a rectangular grid.
    self._alphads = np.asarray(database['alphads'])
    self._betads = np.asarray(database['betads'])

    # databases are generated about a particular omega_hat (normalized pqr)
    self._bomega_hat_0 = np.asarray(database['omega_hat'])

    # Load  the database force coefficients that FBL may or may not utilize.
    self._bf_coef = {
        'cx': np.asarray(database['CXtot'][0]),
        'cy': np.asarray(database['CYtot'][0]),
        'cz': np.asarray(database['CZtot'][0])
        }

    # All database objects are allowed to modifiy force coefficients depending
    # on the body angular rates.
    self._bdf_coef_dpqr = {
        'cxp': np.asarray(database['CXp'][0]),
        'cxq': np.asarray(database['CXq'][0]),
        'cxr': np.asarray(database['CXr'][0]),
        'cyp': np.asarray(database['CYp'][0]),
        'cyq': np.asarray(database['CYq'][0]),
        'cyr': np.asarray(database['CYr'][0]),
        'czp': np.asarray(database['CZp'][0]),
        'czq': np.asarray(database['CZq'][0]),
        'czr': np.asarray(database['CZr'][0])
        }

    # Load  the database moment coefficients that FBL may or may not utilize.
    self._bm_coef = {
        'cl': np.asarray(database['Cltot'][0]),
        'cm': np.asarray(database['Cmtot'][0]),
        'cn': np.asarray(database['Cntot'][0])
        }

    # All database objects are allowed to modifiy force coefficients depending
    # on the body angular rates.
    self._bdm_coef_dpqr = {
        'clp': np.asarray(database['Clp'][0]),
        'clq': np.asarray(database['Clq'][0]),
        'clr': np.asarray(database['Clr'][0]),
        'cmp': np.asarray(database['Cmp'][0]),
        'cmq': np.asarray(database['Cmq'][0]),
        'cmr': np.asarray(database['Cmr'][0]),
        'cnp': np.asarray(database['Cnp'][0]),
        'cnq': np.asarray(database['Cnq'][0]),
        'cnr': np.asarray(database['Cnr'][0])
        }

    # Track any constant database component offsets.
    self._coef_offsets = {
        'cx_0': 0., 'cy_0': 0., 'cz_0': 0., 'cl_0': 0., 'cm_0': 0., 'cn_0': 0.}

  def OffsetBodyForceCoeffs(self, offset_dict):
    """Offsets body force coefficients by specified amounts.

    Args:
      offset_dict: dictionary containing force coefficient offsets.
    """
    #TODO: C-Sim accounts for the matrix body rotation rates for each
    # alpha and beta value. Currently this is just a constant offset, which is
    # a discrepancy. Next step is a full body rotation accounting.
    self._coef_offsets['cx_0'] = offset_dict['CX_0']
    self._coef_offsets['cy_0'] = offset_dict['CY_0']
    self._coef_offsets['cz_0'] = offset_dict['CZ_0']
    self._bf_coef['cx'] += offset_dict['CX_0']
    self._bf_coef['cy'] += offset_dict['CY_0']
    self._bf_coef['cz'] += offset_dict['CZ_0']

  def OffsetBodyMomentCoeffs(self, offset_dict):
    """Offset body moment coefficients by specified amounts.

    Args:
      offset_dict: dictionary containing moment coefficient offsets.
    """
    #TODO: C-Sim accounts for the matrix body rotation rates for each
    # alpha and beta value. Currently this is just a constant offset, which is
    # a discrepancy. Next step is a full body rotation accounting.
    self._coef_offsets['cl_0'] = offset_dict['CL_0']
    self._coef_offsets['cm_0'] = offset_dict['CM_0']
    self._coef_offsets['cn_0'] = offset_dict['CN_0']
    self._bf_coef['cl'] += offset_dict['CL_0']
    self._bf_coef['cm'] += offset_dict['CM_0']
    self._bf_coef['cn'] += offset_dict['CN_0']

  def GetConstantCoeffOffsets(self):
    return self._coef_offsets


class FittedDatabase(CSimDatabase):
  """Uses Bivariate polynomial fits to model the imported C-Sim database.

  Note that when this database instantiates, it does not automatically generate
  fits for the moment coefficients or the roll moment contributions due to body
  rates.
  """

  def __init__(self, filename, fit_method='bivariate'):
    super(FittedDatabase, self).__init__(filename)

    alphas = self._alphads
    betas = self._betads

    if fit_method == 'bivariate':
      self._fitfunc = UseRBSF
    else:
      assert False, 'FittedDatabase fit_method not found'

    self.FitBodyForceCoefficients(alphas, betas)
    self.FitBodyRateForceDerivativeTerms(alphas, betas)
    self.FitBodyMomentCoefficients(alphas, betas)
    self.FitBodyRateMomentDerivativeTerms(alphas, betas)

  def FitBodyForceCoefficients(self, alphas, betas):
    self._bf_coef_fit = {
        'cx': self._fitfunc(alphas, betas, self._bf_coef['cx']),
        'cy': self._fitfunc(alphas, betas, self._bf_coef['cy']),
        'cz': self._fitfunc(alphas, betas, self._bf_coef['cz'])
        }

  def FitBodyRateForceDerivativeTerms(self, alphas, betas):
    self._bdf_coef_dpqr_fit = {
        'cxp': self._fitfunc(alphas, betas, self._bdf_coef_dpqr['cxp']),
        'cxq': self._fitfunc(alphas, betas, self._bdf_coef_dpqr['cxq']),
        'cxr': self._fitfunc(alphas, betas, self._bdf_coef_dpqr['cxr']),
        'cyp': self._fitfunc(alphas, betas, self._bdf_coef_dpqr['cyp']),
        'cyq': self._fitfunc(alphas, betas, self._bdf_coef_dpqr['cyq']),
        'cyr': self._fitfunc(alphas, betas, self._bdf_coef_dpqr['cyr']),
        'czp': self._fitfunc(alphas, betas, self._bdf_coef_dpqr['czp']),
        'czq': self._fitfunc(alphas, betas, self._bdf_coef_dpqr['czq']),
        'czr': self._fitfunc(alphas, betas, self._bdf_coef_dpqr['czr'])
        }

  def FitBodyMomentCoefficients(self, alphas, betas):
    self._bm_coef_fit = {
        'cl': self._fitfunc(alphas, betas, self._bm_coef['cl']),
        'cm': self._fitfunc(alphas, betas, self._bm_coef['cm']),
        'cn': self._fitfunc(alphas, betas, self._bm_coef['cn'])
        }

  def FitBodyRateMomentDerivativeTerms(self, alphas, betas):
    self._bdm_coef_dpqr_fit = {
        'clp': self._fitfunc(alphas, betas, self._bdm_coef_dpqr['clp']),
        'clq': self._fitfunc(alphas, betas, self._bdm_coef_dpqr['clq']),
        'clr': self._fitfunc(alphas, betas, self._bdm_coef_dpqr['clr']),
        'cmp': self._fitfunc(alphas, betas, self._bdm_coef_dpqr['cmp']),
        'cmq': self._fitfunc(alphas, betas, self._bdm_coef_dpqr['cmq']),
        'cmr': self._fitfunc(alphas, betas, self._bdm_coef_dpqr['cmr']),
        'cnp': self._fitfunc(alphas, betas, self._bdm_coef_dpqr['cnp']),
        'cnq': self._fitfunc(alphas, betas, self._bdm_coef_dpqr['cnq']),
        'cnr': self._fitfunc(alphas, betas, self._bdm_coef_dpqr['cnr'])
        }

  def CalcForceCoeffs(self, alphad, betad, omega_hat=np.zeros((3, ))):
    """Computes the body force and moment coefficients from a C-Sim database.

    Args:
      alphad: Angle of attack [deg].
      betad: Side-slip angle [deg].
      pqr_body: Array of 3-by-1 *dimensional* body rates [#].

    Returns:
      A dict containing computed force coefficients.
    """
    cx = self._bf_coef_fit['cx'].ev(alphad, betad)
    cy = self._bf_coef_fit['cy'].ev(alphad, betad)
    cz = self._bf_coef_fit['cz'].ev(alphad, betad)
    cx += self.CalcCXFromOmegaHat(alphad, betad, omega_hat)
    cy += self.CalcCYFromOmegaHat(alphad, betad, omega_hat)
    cz += self.CalcCZFromOmegaHat(alphad, betad, omega_hat)

    return {'cx': cx, 'cy': cy, 'cz': cz}

  def CalcCXFromOmegaHat(self, alphad, betad, omega_hat):
    domh_x = omega_hat[0] - self._bomega_hat_0[0]
    domh_y = omega_hat[1] - self._bomega_hat_0[1]
    domh_z = omega_hat[2] - self._bomega_hat_0[2]
    cx_from_omh = self._bdf_coef_dpqr_fit['cxp'].ev(alphad, betad) * domh_x
    cx_from_omh += (self._bdf_coef_dpqr_fit['cxq'].ev(alphad, betad) * domh_y)
    cx_from_omh += (self._bdf_coef_dpqr_fit['cxr'].ev(alphad, betad) * domh_z)
    return cx_from_omh

  def CalcCYFromOmegaHat(self, alphad, betad, omega_hat):
    domh_x = omega_hat[0] - self._bomega_hat_0[0]
    domh_y = omega_hat[1] - self._bomega_hat_0[1]
    domh_z = omega_hat[2] - self._bomega_hat_0[2]
    cy_from_omh = self._bdf_coef_dpqr_fit['cyp'].ev(alphad, betad) * domh_x
    cy_from_omh += (self._bdf_coef_dpqr_fit['cyq'].ev(alphad, betad) * domh_y)
    cy_from_omh += (self._bdf_coef_dpqr_fit['cyr'].ev(alphad, betad) * domh_z)
    return cy_from_omh

  def CalcCZFromOmegaHat(self, alphad, betad, omega_hat):
    domh_x = omega_hat[0] - self._bomega_hat_0[0]
    domh_y = omega_hat[1] - self._bomega_hat_0[1]
    domh_z = omega_hat[2] - self._bomega_hat_0[2]
    cz_from_omh = self._bdf_coef_dpqr_fit['czp'].ev(alphad, betad) * domh_x
    cz_from_omh += (self._bdf_coef_dpqr_fit['czq'].ev(alphad, betad) * domh_y)
    cz_from_omh += (self._bdf_coef_dpqr_fit['czr'].ev(alphad, betad) * domh_z)
    return cz_from_omh

  def OffsetBodyForceCoeffs(self, offset_dict):
    super(FittedDatabase, self).OffsetBodyForceCoeffs(offset_dict)
    self.FitBodyForceCoefficients(self._alphads, self._betads)

  def OffsetBodyMomentCoeffs(self, offset_dict):
    super(FittedDatabase, self).OffsetBodyMomentCoefs(offset_dict)
    self.FitBodyMomentCoefficients(self._alphads, self._betads)

  def CalcMomentCoeffs(self, alphad, betad, pqr_body=np.zeros((3,))):
    """Calculates the body force and moment coefficients from a C-Sim database.

    Args:
      alphad: Angle of attack [deg].
      betad: Side-slip angle [deg].
      pqr_body: Array of 3-by-1 dimensional body rates [#].

    Returns:
      A dict containing computed moment coefficients.

    """
    cl = self._bm_coef_fit['cl'].ev(alphad, betad)
    cm = self._bm_coef_fit['cm'].ev(alphad, betad)
    cn = self._bm_coef_fit['cn'].ev(alphad, betad)
    cl += self.CalcCLFromOmegaHat(alphad, betad, pqr_body)
    cm += self.CalcCMFromOmegaHat(alphad, betad, pqr_body)
    cn += self.CalcCNFromOmegaHat(alphad, betad, pqr_body)

    return {'cl': cl, 'cm': cm, 'cn': cn}

  def CalcCLFromOmegaHat(self, alphad, betad, omega_hat):
    domh_x = omega_hat[0] - self._bomega_hat_0[0]
    domh_y = omega_hat[1] - self._bomega_hat_0[1]
    domh_z = omega_hat[2] - self._bomega_hat_0[2]
    cl_from_omh = self._bdm_coef_dpqr_fit['clp'].ev(alphad, betad) * domh_x
    cl_from_omh += (self._bdm_coef_dpqr_fit['clq'].ev(alphad, betad) * domh_y)
    cl_from_omh += (self._bdm_coef_dpqr_fit['clr'].ev(alphad, betad) * domh_z)
    return cl_from_omh

  def CalcCMFromOmegaHat(self, alphad, betad, omega_hat):
    domh_x = omega_hat[0] - self._bomega_hat_0[0]
    domh_y = omega_hat[1] - self._bomega_hat_0[1]
    domh_z = omega_hat[2] - self._bomega_hat_0[2]
    cm_from_omh = self._bdm_coef_dpqr_fit['cmp'].ev(alphad, betad) * domh_x
    cm_from_omh += (self._bdm_coef_dpqr_fit['cmq'].ev(alphad, betad) * domh_y)
    cm_from_omh += (self._bdm_coef_dpqr_fit['cmr'].ev(alphad, betad) * domh_z)
    return cm_from_omh

  def CalcCNFromOmegaHat(self, alphad, betad, omega_hat):
    domh_x = omega_hat[0] - self._bomega_hat_0[0]
    domh_y = omega_hat[1] - self._bomega_hat_0[1]
    domh_z = omega_hat[2] - self._bomega_hat_0[2]
    cn_from_omh = self._bdm_coef_dpqr_fit['cnp'].ev(alphad, betad) * domh_x
    cn_from_omh += (self._bdm_coef_dpqr_fit['cnq'].ev(alphad, betad) * domh_y)
    cn_from_omh += (self._bdm_coef_dpqr_fit['cnr'].ev(alphad, betad) * domh_z)
    return cn_from_omh



