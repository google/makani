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

"""Makani configuration system.

Our configuration files describe many aspects of the physical system,
simulator, controller, and monitoring system.  The final form of the
configuration files are either JSON files, which can be loaded at
run-time, or .c files that are compiled in for the default settings.
However, it is useful to generate these files from a more flexible
system that allows simple math and dependencies between different
parameters.  Thus, we use a set of Python functions that return
dictionaries to generate the final configuration files.

A simple example of one of these Python functions that generates a
simplified tether parameter structure where the tether length is
determined by the wing span is shown here:

  from makani.config import mconfig

  @mconfig.Config(deps={'wing', 'm600.wing'})
  def MakeParams(params):
    return {
        # Tether length [m] under zero load.
        'length': 18.0 * params['wing']['b'],

        # Tether and bridle mass [kg].
        'm': 15.65
    }

Then, to generate the Python dictionary from the file containing that function,
run:

  params = mconfig.MakeParams('m600.tether')

More examples of how to use the mconfig module are in
config/write_params.py.
"""

import copy
import ctypes
import importlib
import inspect
import warnings

import gflags
from makani.lib.python import wing_flag
from makani.lib.python.autogen import autogen_util
import numpy as np

# This warning is a bug in Python.  See http://stackoverflow.com/
# questions/4964101/pep-3118-warning-when-using-ctypes-array-as-numpy-array
# for more information.
warnings.filterwarnings('ignore', message='Item size computed from the '
                        'PEP 3118 buffer format string does not match the '
                        'actual item size.')

wing_flag.AppeaseLintWhenImportingFlagOnly()

WING_MODEL = None


class InvalidOverrideException(Exception):
  """Raised by SimpleOverride when an override does not exist."""
  pass


class InvalidOverrideMethodException(Exception):
  """Raised if override method is not 'simple' or 'derived'."""
  pass


def _RelativeImport(module_relpath):
  """Returns a module imported by its path relative to this package."""
  return importlib.import_module('.' + module_relpath, package=__package__)


def _ConvertOverridesToModuleOverrides(current_module, overrides,
                                       module_overrides):
  """Converts overrides to module overrides.

  Some of the overrides may apply to the current module, while others
  apply to the dependencies of the current module.  If the key name of
  one of the overrides matches a dependency name, then this removes
  that (key, value) pair from the overrides dictionary and tries to
  find which module the override applies to by recursively going
  through the modules.  Once it finds the module that the override
  refers to, it adds the (key, value) pair to a dictionary of "module
  overrides", where the key is the module name and the value is the
  override.

  Args:
    current_module: Relative path to the module
        (e.g. 'all_params.sim.phys_sim').
    overrides: In/out parameter used to store value of override or
        dict of overrides.  Overrides that apply to modules are
        removed from this dict.
    module_overrides: In/out parameter of "module overrides" which is
        a dict where the key is the module name and the value is the
        override.
  """
  params_func = getattr(_RelativeImport(current_module), 'MakeParams')
  argspec = inspect.getargspec(params_func)
  deps = argspec.defaults[argspec.args.index('deps')]
  if isinstance(overrides, dict):
    for key, value in overrides.items():
      if deps and key in deps:
        _ConvertOverridesToModuleOverrides(deps[key], overrides.pop(key),
                                           module_overrides)
      else:
        if current_module not in module_overrides:
          module_overrides[current_module] = {}
        module_overrides[current_module][key] = value

  else:
    module_overrides[current_module] = overrides


def Config(*args, **kwargs):
  # TODO: The implementation here may be changed in the near future.
  # Tighten up the docstrings after that's done, or once we decide to leave this
  # as is.
  # pylint: disable=g-doc-return-or-yield,g-doc-args,missing-docstring
  """Decorator for configuration functions.

  Makani's configuration files are Python scripts that return a dictionary that
  matches the equivalent C structure.  All configuration functions should be
  named MakeParams and have a @Config decorator.  If the configuration file has
  no dependencies then a plain @Config decorator is used.  If the file does have
  dependencies then a @Config(deps={'dep_name': 'dep_module'}) decorator is
  used.

  E.g.
    @Config(deps={'wing': 'm600.wing'})

  Args:
    deps: Dictionary of parameter names and the modules used to
        create this parameter.
  """
  def WrapMakeParams(make_params_func):
    assert make_params_func.__name__ == 'MakeParams'

    def WrappedMakeParams(deps=kwargs.get('deps', None),
                          overrides=None,
                          module_overrides=None):
      if deps is None: deps = {}
      if overrides is None: overrides = {}
      if module_overrides is None: module_overrides = {}

      if deps:
        # Make params_in dictionary from the other configuration files
        # listed in the dependencies.  Pass the module overrides in as
        # standard overrides.
        params_in = {}
        for dep_name, dep_module in deps.iteritems():
          make_dep_params_func = getattr(_RelativeImport(dep_module),
                                         'MakeParams')
          params_in[dep_name] = copy.deepcopy(
              make_dep_params_func(
                  overrides=module_overrides.get(dep_module, None),
                  module_overrides=module_overrides))
        params_out = make_params_func(params_in)

        # Check that a field with the same name as a dependency is
        # equal to the dependency.  This protects against conflicts in
        # the names of override modules and the parameters themselves.
        if isinstance(params_out, dict):
          for dep_name in deps:
            if dep_name in params_out:
              assert params_out[dep_name] is params_in[dep_name]
      else:
        params_out = make_params_func()

      return SimpleOverride(overrides, params_out)

    return WrappedMakeParams

  # Checks whether Config is being called on a single function as it
  # would be for the simple @Config case.  Otherwise, assume it's
  # being called as the @Config(deps=...) case.
  if len(args) == 1 and hasattr(args[0], '__call__'):
    return WrapMakeParams(args[0])
  else:
    return WrapMakeParams


def SimpleOverride(overrides, params):
  """Overrides parameters without rerunning the configuration scripts.

  Args:
    overrides: Input containing the modified values of parameters.
        For dicts, it is only necessary to include the fields that
        will be modified (e.g. overrides = {'wing': {'A': 22.0}} will
        override the wing's area while not affecting other wing
        properties such as mass).  For other types, the types of the
        overrides and params must match.
    params: The parameters to be modified.

  Returns:
    The modified "params" argument.

  Raises:
    InvalidOverrideException: Override parameter does not exist.
  """
  if isinstance(overrides, dict):
    for override_name, override_value in overrides.iteritems():
      if isinstance(params, dict) and not params.has_key(override_name):
        raise InvalidOverrideException(
            'Invalid override; field does not exist: %s' % override_name)
      if isinstance(params, list):
        if not isinstance(override_name, int) and not (isinstance(
            override_name, (str, unicode)) and override_name.isdigit()):
          raise InvalidOverrideException(
              'Invalid override; bad index %s into list' % override_name)
        else:
          override_name = int(override_name)

        if override_name not in range(len(params)):
          raise InvalidOverrideException(
              'Invalid override; bad index %s into list of length %d'
              % (override_name, len(params)))

      if isinstance(override_value, dict):
        SimpleOverride(override_value, params[override_name])
      else:
        # If the type used in the .py file is np.float64, then any type of float
        # is fine for the override value.
        expected_type = type(params[override_name])
        if expected_type == np.float64:
          expected_type = float

        if isinstance(override_value, expected_type):
          params[override_name] = override_value
        else:
          raise InvalidOverrideException(
              'Invalid override; types do not agree '
              '(expected %s, actual %s): %s'
              % (type(params[override_name]), type(override_value),
                 override_name))
    return params
  else:
    if not isinstance(overrides, type(params)):
      raise InvalidOverrideException('Invalid override: %s' % overrides)
    elif (isinstance(overrides, list) and overrides
          and isinstance(overrides[0], dict)):
      # For lists of dictionaries, override values element-by-element.
      if len(overrides) != len(params):
        raise InvalidOverrideException('Invalid override: %s' % overrides)
      else:
        for i in range(len(overrides)):
          params[i] = SimpleOverride(overrides[i], params[i])
        return params
    else:
      return overrides


def MakeParams(base_params, overrides=None, override_method='simple'):
  """Creates a parameters dictionary.

  Given the root of the parameters files, this generates a Python
  dictionary with the calculated parameters.  The wing model corresponding
  to the root file must agee with the --wing_model flag.

  It also allows parameters to be overridden by passing in a dictionary
  of overrides.  For example:

    MakeParams('m600.wing', overrides={'A': 22.2})

  Parameters may be overridden in a 'simple' or 'derived' manner.
  During a 'simple' override, the parameter is only overridden in that
  specific file and has no effect on other parameter files.  During a
  'derived' override, the parameter is override in all configuration
  files that include the parameter.

  Args:
    base_params: The configuration file for the root of the parameters
        dictionary.
    overrides: Dictionary of parameters to override and their new values.
    override_method: Whether we override the parameters only in the
        local file, 'simple', or in all files that use the parameter,
        'derived'.

  Returns:
    Dictionary of the calculated parameters.

  Raises:
    InvalidOverrideMethodException: Override_method is not 'simple' or
        'derived'.
  """
  if overrides is None: overrides = {}

  global WING_MODEL
  if WING_MODEL is None:
    WING_MODEL = gflags.FLAGS.wing_model

  params_func = getattr(_RelativeImport(base_params), 'MakeParams')

  if override_method == 'simple':
    return SimpleOverride(overrides, params_func())

  elif override_method == 'derived':
    module_overrides = {}
    _ConvertOverridesToModuleOverrides(base_params, overrides, module_overrides)
    return params_func(overrides=overrides, module_overrides=module_overrides)

  else:
    raise InvalidOverrideMethodException(
        "Override_method must be 'simple' or 'derived'.")


def IsStrictlyIncreasing(lst):
  """Returns true if the list is strictly increasing."""
  return all([x < y for x, y in zip(lst, lst[1:])])


def DiffDict(dict1, dict2):
  """Finds the differences between two dictionaries.

  NOTE: This function currently isn't used anywhere, but
  it is very useful for debugging differences in parameter
  dictionaries.

  Args:
    dict1: First dictionary to compare.
    dict2: Second dictionary to compare.

  Returns:
    A dictionary of the fields that didn't match with tuples of the
    values from each dictionary.
  """
  diff = {}
  for k in dict1.keys():
    if not dict2.has_key(k):
      diff[k] = (dict1[k], None)
    elif dict1[k] != dict2[k]:
      if isinstance(dict1[k], dict) and isinstance(dict2[k], dict):
        diff[k] = DiffDict(dict1[k], dict2[k])
      else:
        diff[k] = (dict1[k], dict2[k])
  for k in dict2.keys():
    if not dict1.has_key(k):
      diff[k] = (None, dict2[k])

  return diff


def MatchesCStruct(pytype, pyclass, parent=''):
  """Checks if the Python type matches an equivalent C type.

  This will display in bold any fields that are in one of pytype and
  pyclass but not the other.

  Args:
    pytype: Python dictionary to be compared to a C struct.
    pyclass: ctypes Python class equivalent to a C struct.
    parent: A string used in recursive calls to the function to keep
        track of the parent fields so they may be displayed.

  Returns:
    True if the dictionary and ctypes Python class match,
    otherwise False.
  """
  def Bold(s):
    return '\033[1m' + s + '\033[0m'

  if hasattr(pyclass, '_fields_'):
    fields = autogen_util.GetCFields(pyclass)
    # If we are at a leaf struct (like Vec3 or Quat), the pytype can
    # use an array rather than a dictionary.
    if (isinstance(pytype, list) and len(pytype) == len(fields)
        and all([c == ctypes.c_double for (_, c) in fields])):
      all_floats = True
      for i in range(len(pytype)):
        if not isinstance(pytype[i], float):
          print Bold('%s[%d] must be a float.' % (parent, i))
          all_floats = False
      return all_floats

    diff = set(pytype) ^ {k for (k, _) in fields}
    if diff:
      print Bold(parent + '.{' + ', '.join(diff) + '}') + ' are different.'
      return False
    for k, c in fields:
      if not MatchesCStruct(pytype[k], c, parent + '.' + k):
        return False
    return True

  elif hasattr(pyclass, '_length_'):
    if isinstance(pytype, str):
      if getattr(pyclass, '_type_') is not ctypes.c_char:
        print Bold(parent) + ' should not be a string.\n'
        return False
      elif len(pytype) + 1 > getattr(pyclass, '_length_'):
        # Strings can be shorter than the allotted array (but require
        # room for null termination).
        print Bold(parent) + ' has the wrong length.\n'
        return False
      else:
        return True
    else:
      expected_length = getattr(pyclass, '_length_')
      try:
        actual_length = len(pytype)
      except TypeError:
        print (Bold(parent) +
               ' is expected to be an array of length %d.' % expected_length)
        return False
      if actual_length != expected_length:
        print (Bold(parent) +
               ' has the wrong length (expected %d, actual %d).\n' %
               (expected_length, actual_length))
        return False
      else:
        expected_type = getattr(pyclass, '_type_')
        return all([
            MatchesCStruct(pytype[i], expected_type, '%s[%d]' % (parent, i))
            for i in xrange(len(pytype))])

  else:
    # Check that scalar types are properly specified.
    if pyclass == ctypes.c_double and not isinstance(pytype, float):
      print Bold(parent + ' must be a float.')
      return False
    elif pyclass == ctypes.c_int and not isinstance(pytype, int):
      print Bold(parent + ' must be an int.')
      return False
    elif pyclass == ctypes.c_bool and not isinstance(pytype, bool):
      print Bold(parent + ' must be a bool.')
      return False
    return True
