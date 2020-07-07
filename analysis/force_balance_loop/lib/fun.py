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

from __future__ import absolute_import
from __future__ import print_function
import os
import json
from numbers import Number
import numpy as np
import math
import matplotlib.pyplot as plt
import power_calcs
from six.moves import map
from six.moves import range
from six.moves import zip


def GetFullPath(rel_path):
  """Returns the full path of a file, given its relative path in module."""
  base_dir = os.path.dirname(power_calcs.__file__)
  return os.path.join(base_dir, '../', rel_path)

def map_dict(d, fn):
  """Recursively does operations to all values in a nested dictionary and
  returns a new dictionary of the result.

  Arguments: \n
  d = dictionary to do operations on \n
  fn = function to apply\n"""
  o = {}
  for key, value in d.items():
    if isinstance(value, dict):
      o[key] = map_dict(value, fn)
    else:
      o[key] = fn(value)
  return o

def newton_raphson_solve(
  fnc, init_guess, init_increment=0.01, tol=0.0001, slope_tol=1e-9,
  iter_max=8, fnc_params=(), bounds=None, verbose=False, return_last=False,
  return_converge_data=False):
  """Returns root for provided function using newton-raphson method.

  Returns None if tolerance is not met within number of iterations.

  Args:
    fnc: function to find root of
    init_guess: initial guess of root
    init_increment: relative change of init guess for initial increment
    tol: relative change of root for convergence
    iter_max: maximum number of iterations
    fnc_params: tuple of other params to pass into function
    bounds: tuple of limits for guesses
    verbose: prints convergence messages
  """
  x_new = None

  error_old = None
  error_new = None
  error_slope = None
  converge_data = []

  # Find error of initial guess
  x_old = init_guess
  # Get the first derivative
  error_old = fnc(x_old, *fnc_params)

  x_new = x_old + init_increment * init_guess
  error_new = fnc(x_new, *fnc_params)

  iteration = 0
  converged = False
  for iteration in range(iter_max):
    if x_new == x_old:
      error_slope = 0.
    else:
      error_slope = (error_new - error_old)/(x_new - x_old)
    if abs(error_slope) < slope_tol:
      if verbose:
        print('Did not converge. Slope not in tolerance on iteration %d' % iteration)
        print('Current slope is %0.3g, limit is %0.3g' % (error_slope, slope_tol))
      if not return_last:
        x_new = None
      break
    x_old = x_new
    error_old = error_new
    x_new = x_new - error_new / error_slope
    if bounds is not None:
      if x_new <= bounds[0]:
        x_new = bounds[0]
        if x_new == x_old:
          if verbose:
            print('Value of %0.4g is at lower bound of %0.4g' % (x_new, bounds[0]))
          converged = False
          break
      if x_new >= bounds[1]:
        x_new = bounds[1]
        if x_new == x_old:
          if verbose:
            print('Value of %0.4g is at upper bound of %0.4g' % (x_new, bounds[1]))
          converged = False
          break

    error_new = fnc(x_new, *fnc_params)
    if return_converge_data:
      converge_data.append((x_old, x_new, error_old, error_new, error_slope))

    if abs(error_new) < tol:
      converged = True
      if verbose == True:
        print('Converged to %0.4g in %d iterations. Tolerance is %0.4g' % (error_new, iteration, tol))
      break
  if not converged:
    if not return_last:
      x_new = None
    if verbose == True:
      print('Did not converge within tolerance of %f and iteration limit of %d' % (tol, iter_max))
      print('Current error is %0.4g' % error_new)
  if return_converge_data:
    return x_new, converge_data
  else:
    return x_new


def mask_lists(mask_list, oper_list, mask_val=None, replace_val=None):
  o = []
  for x, y in zip(mask_list, oper_list):
    if x is mask_val:
      y = replace_val
    o.append(y)
  return o


def strip_nones(lst):
  return [y for y in lst if y is not None]


def replace_nones(lst, replace_with=0.):
  def fix_none(x):
    if x is None:
      x = replace_with
    return x
  return list(map(fix_none, lst))

def weib_cdf(x, n, k):  # cdf
  """Provides a weibull cfd dist

    Arguments: \n
    x = the value you want to know the weibull curve for
    n = alpha or the scale factor
    k = shape factor
  """
  return (1 - np.exp(-(x / n) ** k))


def weib_pdf(x,n,a): #pdf
  return (a / n) * (x / n)**(a - 1) * np.exp(-(x / n)**a)


def nested_dict_sum(vals, x = 0.0):
  """recursively sums all values in nested dictionaries \n
     use x to give initial value"""
  if isinstance(vals, dict):
    try:
      x += sum(vals.values())
    except:
      for dct in vals.values():
        x += nested_dict_sum(dct)

  elif isinstance(vals, float) or isinstance(vals, int):
    x += vals
  elif hasattr(vals, '__iter__'):
    for i in vals:
      x += nested_dict_sum(i)
  else:
    print("issue with nested_dict_sum dict is not as expect ")

  return x


def nested_dict_flatten(d):
  o = {}
  if hasattr(d, 'items'):
    for key, value in d.items():
      if hasattr(value, 'items'):
        o.update(nested_dict_flatten(value))
      else:
        o.update({key:value})
    return o


def nested_dict_flatten_w_path(d, path=''):
  o = {}
  if hasattr(d, 'items'):
    for key, value in d.items():
      new_path = path + str(key)
      if hasattr(value, 'items'):
        new_path += '_'
        o.update(nested_dict_flatten_w_path(value, new_path))
      else:
        o.update({new_path:value})
    return o


def nested_dict_lookup(key, var):
  """Searches a nested dictionary for a key and returns the first key it finds.

  Returns None if not found.
  Warning: if key is in multiple nest levels,
    this will only return one of those values."""
  o = None
  if hasattr(var, 'iteritems'):
    for k, v in var.items():
      if k == key:
        o = v
        break
      if isinstance(v, dict):
        result = nested_dict_lookup(key, v)
        if result:
          o = result
  return o


def nested_dict_var_dicts(var, var_list=[]):
  """Searches a nested dictionary for a key and returns the first key it finds.

  Returns None if not found.
  Warning: if key is in multiple nest levels,
    this will only return one of those values."""

  if hasattr(var, 'iteritems'):
    for k, v in var.items():
      if 'nom' in v:
        var_list.append(str(k))
        break
      if isinstance(v, dict):
        var_list = nested_dict_var_dicts(var, var_list)
  return var_list


def nested_dict_get_path(key, var):
  """Searches a nested dictionary for a key and returns
  a list of strings designating the
  complete path to that key.

  Returns an empty list if key is not found.
  Warning: if key is in multiple nest levels,
  this will only return one of those values."""

  path = []
  if hasattr(var, 'iteritems'):
    for k, v in var.items():
      if k == key:
        path.append(k)
        break
      if isinstance(v, dict):
        local_path = [k]
        maybe_path = nested_dict_get_path(key, v)
        if maybe_path != []:
          local_path.extend(maybe_path)
          path.extend(local_path)
  return path


def nested_dict_make_path(path, d):
  """Creates nested path in d and sets value to None.
  Does not overwrite things that already exist.

  Works for traversing a list within the dict via specifying index in path."""
  val = d
  for p in path:
    # Val may be a list, not a dict. If we check to see if p is in val, we
    # can't traverse lists. So we check this by just trying it.
    try:
      val = val[p]
    except:
      if p != path[-1]:
        val[p] = {}
      else:
        val[p] = None
      val = val[p]


def nested_dict_traverse(path, d):
  """Uses path to traverse a nested dictionary and
  returns the value at the end of the path.

  Assumes path is valid, will return KeyError if it is not.
  Warning: if key is in multiple nest levels, 
  this will only return one of those values."""
  val = d
  for p in path:
    val = val[p]
  return val


def nested_dict_update(path, update_val, d, make_path=False):
  """Updates the item at the end of the path in dictionary d with update_val.

  Arguments:
    path: list of strings indicating path to
      variable getting updated in a nested dictionary. Lists can be traversed by
      path by specifying index rather than key.
    update_val: the value to put into the nested dictionary
    d: the nested dictionary that gets updated.
    make_path: If path isn't present in d, make the path."""

  if make_path:
    nested_dict_make_path(path, d)
  val = d
  for p in path[:-1]:
    val = val[p]
  key = path[len(path)-1]
  val[key] = update_val


def nested_dict_val_remove(path, d):
  """Removes the item at the end of the path in dictionary d.

  Arguments: \n
    path: list of strings indicating path to
    variable getting removed in a nested dictionary
    d: the nested dictionary that gets changed"""
  val = d
  for p in path[:-1]:
    val = val[p]
  key = path[len(path)-1]
  val.pop(key, None)


def nested_dict_strip_to_json(d, bool_to_str=False):
  """Returns a new copy of a nested dict prepared for encoding to JSON. Does not
  save to JSON, just prepares it.

  Nested dict can contain any dict-like or list-like entries.
  Anything not saveable is stripped out.

  Kwargs:
    bool_to_str: Saves boolean values as a string. Ie: True --> "True"
      This to make it easier for use cases where all output needs to be easily
      string-ified.
  """
  # Make an encoder to test.
  encoder = json.JSONEncoder()

  o = None

  # If it can be turned into a list, do so now. Catches cases of 0d arrays.
  d_temp = d.tolist() if hasattr(d, 'tolist') else d
  try:
    len(d_temp)
  except TypeError:
    o = d_temp

  #TODO: Refactor to clean up.
  # Check if dict-like.
  if hasattr(d_temp, 'items'):
    o = {}
    for key, value in d_temp.items():
      if ((hasattr(value, 'items') or hasattr(value, '__iter__'))
            and not isinstance(value, str)):
        o[key] = nested_dict_strip_to_json(value)
      else:
        try:
          test = encoder.encode(value)
          if type(value) is not bool:
            v_temp = value
            if v_temp == float('inf') or value == float('-inf'):
              v_temp = None
          else:
            v_temp = value if not bool_to_str else str(value)
          o[key] = v_temp
        except TypeError:
          if hasattr(value, 'tolist'):
            o[key] = nested_dict_strip_to_json(value.tolist())
  # Check if list-like.
  elif hasattr(d_temp, '__iter__'):
    o = []
    for value in d_temp:
      if ((hasattr(value, 'items') or hasattr(value, '__iter__'))
            and not isinstance(value, str)):
        o.append(nested_dict_strip_to_json(value))
      else:
        try:
          test = encoder.encode(value)
          if type(value) is not bool:
            v_temp = value
            if v_temp == float('inf') or value == float('-inf'):
              v_temp = None
          else:
            v_temp = value if not bool_to_str else str(value)

          o.append(v_temp)
        except TypeError:
          # Catch the case where it's dict like, but is a numpy array.
          # Convert this case to a list.
          if hasattr(value, 'tolist'):
            o.append(nested_dict_strip_to_json(value.tolist()))
  return o


def stat_dict(lst):
  o = {}
  o['min'] = np.min(lst)
  o['avg'] = np.mean(lst)
  o['max'] = np.max(lst)
  o['std'] = np.std(lst)

  return o


def convert_keys_to_string(dictionary):
    """Recursively converts dictionary keys to strings."""
    if not isinstance(dictionary, dict):
        return dictionary
    return dict((str(k), convert_keys_to_string(v))
        for k, v in dictionary.items())


def calc_AEP(prob_v_w_dist, power_curve, p_rated, losses_pct={'elec':0.0}, system_loss_energy=0.0,
             system_loss_hrs=0.0, v_w_in=0., v_w_out=25.):
  '''Calculates the annual energy production per kite for a given power curve, 
  v_w distribution at hub height, and losses.
  
  losses_pct needs to be a dictionary, with at least one part being 'elec', for collection system
  electrical losses.'''
  
  # AEP with no losses
  power_prob_array = []
  power_prob_norm_array = []
  
  power_avg_per_sys_no_loss = 0.0
  
  powers = power_curve['powers_positive_in_limits']
  powers = fun.replace_nones(powers)
  v_ws_at_h_hub = power_curve['v_ws_at_h_hub']
  cdfs_v_w_at_h_ref = prob_v_w_dist['cdf_v_w_at_h_ref']

  prev_power = powers[0]
  prev_v_w_at_h_hub = v_ws_at_h_hub[0]
  prev_cdf_at_h_ref = cdfs_v_w_at_h_ref[0]
  
  # Multiply power curve times wind probability at hub
  for power, v_w_at_h_hub, cdf_v_w_at_h_ref in zip(powers, v_ws_at_h_hub, cdfs_v_w_at_h_ref):

    if v_w_at_h_hub > v_w_in and prev_v_w_at_h_hub < v_w_in:
      cdf_lin = interp1d([prev_v_w_at_h_hub, v_w_at_h_hub],
                      [prev_cdf_at_h_ref, cdf_v_w_at_h_ref])
      prob = cdf_v_w_at_h_ref - cdf_lin(v_w_in)

      p_lin = interp1d([prev_v_w_at_h_hub, v_w_at_h_hub],
                       [prev_power, power])
      p_avg = (power + p_lin(v_w_in))/2.

    elif v_w_at_h_hub > v_w_out and prev_v_w_at_h_hub < v_w_out:
      cdf_lin = interp1d([prev_v_w_at_h_hub, v_w_at_h_hub],
                      [prev_cdf_at_h_ref, cdf_v_w_at_h_ref])
      prob = cdf_lin(v_w_out) - prev_cdf_at_h_ref

      p_lin = interp1d([prev_v_w_at_h_hub, v_w_at_h_hub],
                       [prev_power, power])
      p_avg = (prev_power + p_lin(v_w_out))/2.

    else:
      prob = cdf_v_w_at_h_ref - prev_cdf_at_h_ref
      if v_w_at_h_hub < v_w_in or v_w_at_h_hub > v_w_out:
        p_avg = 0.
      else:
        p_avg = (power + prev_power)/2.

    power_prob = (prob * p_avg)
    power_prob_array.append(power_prob)

    power_prob_norm_array.append(power_prob / p_rated)
    power_avg_per_sys_no_loss += power_prob
    
    prev_power = power
    prev_v_w_at_h_hub = v_w_at_h_hub
    prev_cdf_at_h_ref = cdf_v_w_at_h_ref

  AEP_no_losses = (power_avg_per_sys_no_loss * const.hrs_per_yr / 1000000.0)

  
  prob_power = {'v_ws_at_h_ref': power_curve['v_ws_at_h_ref'],
                'power_prob': power_prob_array,
                'power_prob_norm': power_prob_norm_array}
  
  cum_losses = 1.0
  for loss in losses_pct.values():
    cum_losses *= (1.0 - loss)
  losses_pct_total = 1.0 - cum_losses
  
  # AEP with losses
  energy_gen_per_yr = (power_avg_per_sys_no_loss * (const.hrs_per_yr - system_loss_hrs)
                       * (1.0 - losses_pct_total))

  energy_loss_per_yr = (system_loss_energy / (1.0 - losses_pct['elec']))

  AEP = (energy_gen_per_yr - energy_loss_per_yr) / 1000000.0

  return AEP, AEP_no_losses, prob_power
