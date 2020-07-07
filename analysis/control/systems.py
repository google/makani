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

"""Functions for working with state-space models."""


import copy

import numpy as np


# pylint doesn't like capital letters in variable names, in contrast
# to control systems conventions.
# pylint: disable=invalid-name


class SignalListInvalidArgumentException(Exception):
  """Raised if an invalid argument is passed to a method of SignaList."""
  pass


class SignalList(object):
  """Representation of a named list of signals.

  A signal list consists of a non-repeating ordered set of signal
  names (each representing a scalar valued signal).
  """

  def __init__(self, names):
    """Constructor.

    Args:
      names: List of signal names.

    Raises:
      SignalListInvalidArgumentException: If there are repeated names.
    """
    self.names = copy.copy(names)
    if not all([isinstance(name, str) for name in self.names]):
      raise SignalListInvalidArgumentException('Names must be strings.', names)
    # Test that there are not repeated names.
    if len(self.names) != len(set(self.names)):
      raise SignalListInvalidArgumentException('Repeated signal name.', names)
    self._names_to_indices = {
        name: i for i, name in enumerate(self.names)
    }

  def __repr__(self):
    return ', '.join(self.names)

  def __getitem__(self, indices):
    """Select a subset of an input list.

    Args:
      indices: Can be a slice or a list.  If a list, may contain
          a mixture of integer entries and string entries.

    Raises:
      SignalListInvalidArgumentException: If indices is not a list.

    Returns:
      A new signal list consisting of a subset of the existing list
      in the given order (units are tracked appropriately).
    """
    if isinstance(indices, slice):
      selection = range(len(self))[indices]
    elif not isinstance(indices, list):
      raise SignalListInvalidArgumentException(indices)
    else:
      selection = copy.copy(indices)

    for i in range(len(selection)):
      if isinstance(selection[i], int):
        selection[i] = self.names[selection[i]]

    return SignalList(selection)

  def __len__(self):
    return len(self.names)

  def __add__(self, other):
    return SignalList(self.names + other.names)

  def GetIndices(self, names):
    """Get the integer indices corresponding to a list of signal names."""
    return [self._names_to_indices[name] for name in names]

  def AddSuffix(self, suffix):
    return SignalList([name + suffix for name in self.names])


class SystemInvalidArgumentException(Exception):
  """Raised if an invalid argument is passed to a method of System."""
  pass


class SystemBadDimensionException(Exception):
  """Raised if there is a mismatch in dimensions of arguments."""
  pass


class SystemSamplePeriodMismatchException(Exception):
  """Raised if there is a mismatch in sample periods."""
  pass


class SystemBadSamplePeriodException(Exception):
  """Raised if an illegal sample period is given."""
  pass


class SystemIllPosedException(Exception):
  """Raised if an ill-posed feedback loop is closed."""
  pass


class System(object):
  """Class for representing as state-space model with named inputs."""

  def __init__(self, A, B, C, D, Ts, state_list, input_list, output_list):
    """Constructor for a new system.

    If A, B, or C are empty, then all three must be empty and
    the system represents a constant gain matrix determined
    by D.

    If A is non-empty and D is empty, then D is taken to be all zeros.

    Args:
      A: nx-by-nx matrix.
      B: nx-by-nu matrix.
      C: ny-by-nx matrix.
      D: ny-by-nu matrix.
      Ts: Sample period (zero indicates continuous time models, -1.0
          indicates a DT model with unspecified sample period).
      state_list: SignalList of length nx.
      input_list: SignalList of length nu.
      output_list: SignalList of length ny.

    Raises:
      SystemBadSamplePeriodException: If Ts has an invalid value.
      SystemBadDimensionException: If the arguments have inconsistent
          dimensions.
    """
    if not state_list:
      state_list = SignalList([])
    if isinstance(state_list, list):
      state_list = SignalList(state_list)
    if isinstance(input_list, list):
      input_list = SignalList(input_list)
    if isinstance(output_list, list):
      output_list = SignalList(output_list)
    if Ts < 0.0 and Ts != -1.0:
      raise SystemBadSamplePeriodException()
    self.Ts = Ts

    self._A = np.matrix(A)
    self._B = np.matrix(B)
    self._C = np.matrix(C)
    self._D = np.matrix(D)
    if self._A.size == 0:
      if self._B.size > 0 or self._C.size > 0:
        raise SystemBadDimensionException(A, B, C, D)
      self.nx = 0
      self.ny = self._D.shape[0]
      self.nu = self._D.shape[1]
      self._A = np.matrix(np.zeros((self.nx, self.nx)))
      self._B = np.matrix(np.zeros((self.nx, self.nu)))
      self._C = np.matrix(np.zeros((self.ny, self.nx)))
    else:
      self.nx = self._A.shape[0]
      self.nu = self._B.shape[1]
      self.ny = self._C.shape[0]
    if self._D.size == 0:
      self._D = np.matrix(np.zeros((self.ny, self.nu)))

    if (not np.array_equal(self._A.shape, [self.nx, self.nx])
        or not np.array_equal(self._B.shape, [self.nx, self.nu])
        or not np.array_equal(self._C.shape, [self.ny, self.nx])
        or not np.array_equal(self._D.shape, [self.ny, self.nu])):
      raise SystemBadDimensionException(self._A.shape, self._B.shape,
                                        self._C.shape, self._D.shape)

    if not isinstance(state_list, SignalList) or len(state_list) != self.nx:
      raise SystemBadDimensionException('Bad state list.', state_list)

    if not isinstance(input_list, SignalList) or len(input_list) != self.nu:
      raise SystemBadDimensionException('Bad input list.', input_list)

    if not isinstance(output_list, SignalList) or len(output_list) != self.ny:
      raise SystemBadDimensionException('Bad output list.', output_list)

    self.states = state_list
    self.inputs = input_list
    self.outputs = output_list

  def __repr__(self):
    return ('%d-by-%d state-space model (Ts = %g) with %d states.' %
            (self.ny, self.nu, self.Ts, self.nx)
            + '\nInputs: ' + self.inputs.__repr__()
            + '\nOutputs: ' + self.outputs.__repr__()
            + '\nA:\n' + str(self._A) + '\nB:\n' + str(self._B)
            + '\nC:\n' + str(self._C) + '\nD:\n' + str(self._D))

  def __getitem__(self, io):
    """Construct a new system with a subset of the inputs and outputs.

    Args:
      io: A tuple of two lists of strings.  The first list selects a subset
        of the outputs of the system, the second selects a subset of the inputs.
        Either can be the trivial slice ':'.

    Raises:
      SystemInvalidArgumentException: If an invalid argument is passed.

    Returns:
      A new System object with the reduced input and output dimensions.
    """
    if not isinstance(io, tuple) or len(io) != 2:
      raise SystemInvalidArgumentException('Index must be a tuple.')

    if isinstance(io[1], slice):
      if io[1] != slice(None, None, None):
        raise SystemInvalidArgumentException('Only ":" slices are allowed.')
      input_names = [self.inputs.names[i] for i in range(self.nu)]
    else:
      input_names = io[1]

    if isinstance(io[0], slice):
      if io[0] != slice(None, None, None):
        raise SystemInvalidArgumentException('Only ":" slices are allowed.')
      output_names = [self.outputs.names[i] for i in range(self.ny)]
    else:
      output_names = io[0]

    (inputs, _, B, _) = self._PartitionInputs(self._B, input_names)
    (outputs, _, C, _) = self._PartitionOutputs(self._C, output_names)
    (_, _, D, _) = self._PartitionInputs(self._D, input_names)
    (_, _, D, _) = self._PartitionOutputs(D, output_names)

    return System(self._A, B, C, D, self.Ts, self.states, inputs, outputs)

  def _PartitionOutputs(self, matrix, output_names):
    """Partition the rows of an ny-by-m matrix.

    Args:
      matrix: An ny-by-m matrix.
      output_names: Names of the outputs to be included in the first
          part of the partition.

    Raises:
      SystemBadDimensionException: If matrix has the wrong number of rows.

    Returns:
      A tuple (outputs, other_outputs, output_matrix,
      other_output_matrix).  The first two entries are SignalLists
      containing the outputs in output_names and the other outputs.
      The first matrix is the sub-matrix corresponding to the
      output_names and the remaining rows are in other_output_matrix.
    """
    if matrix.shape[0] != self.ny:
      raise SystemBadDimensionException(matrix)

    outputs = self.outputs[output_names]
    output_indices = self.outputs.GetIndices(output_names)
    other_output_indices = [
        i for i in range(self.ny) if i not in output_indices
    ]
    other_outputs = self.outputs[other_output_indices]

    output_matrix = matrix[output_indices, :]
    other_output_matrix = matrix[other_output_indices, :]

    return (outputs, other_outputs, output_matrix, other_output_matrix)

  def _PartitionInputs(self, matrix, input_names):
    """Partition the columns of an m-by-nu matrix.

    Args:
      matrix: An m-by-nu matrix.
      input_names: Names of the inputs to be included in the first
          part of the partition.

    Raises:
      SystemBadDimensionException:  If matrix has the wrong number of columns.

    Returns:
      A tuple (inputs, other_inputs, input_matrix,
      other_input_matrix).  The first two entries are SignalLists
      containing the inputs in input_names and the other inputs.
      The first matrix is the sub-matrix corresponding to the
      input_names and the remaining rows are in other_input_matrix.
    """
    if matrix.shape[1] != self.nu:
      raise SystemBadDimensionException(matrix)

    inputs = self.inputs[input_names]
    input_indices = self.inputs.GetIndices(input_names)
    other_input_indices = [
        i for i in range(self.nu) if i not in input_indices
    ]
    other_inputs = self.inputs[other_input_indices]

    input_matrix = matrix[:, input_indices]
    other_input_matrix = matrix[:, other_input_indices]

    return (inputs, other_inputs, input_matrix, other_input_matrix)

  def GetStateSpaceModel(self):
    """Return the state-space description of the system."""
    return self._A, self._B, self._C, self._D, self.Ts

  def ReduceStates(self, state_names):
    """Returns a new system model that keeps only a subset of the states.

    Args:
      state_names: List of state names to retain.

    Returns:
      A new state-space model with only the desired states retained.
    """
    state_indices = self.states.GetIndices(state_names)
    states = self.states[state_names]

    A = self._A[[[s] for s in state_indices], state_indices]
    B = self._B[[s for s in state_indices], :]
    C = self._C[:, state_indices]

    return System(A, B, C, self._D, self.Ts, states, self.inputs, self.outputs)
