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

"""Classes to convert Gradebook to checklist items."""

from makani.analysis.checks import base_check
from makani.lib.python.h5_utils import numpy_utils
import numpy

# Default window size to measure value statistics such as standard deviation.
GRADEBOOK_FILTER_WINDOW_SIZE = 100
GRADEBOOK_FILTER_WINDOW_STEP = GRADEBOOK_FILTER_WINDOW_SIZE / 2


class GradebookItem(base_check.BaseCheckItem):
  """A checklist item created from the gradebook."""

  @base_check.RegisterSpecs
  def __init__(self, name, data_index, task=None, *args):
    """Initialize a gradebook item.

    Args:
      name: Name of the check.
      data_index: index to the data source. E.g. MotorStatus.MotorSbi.omega.
      task: A callback function to preprocess the input.
      *args: Please refer to BaseCheckItem.

    Raises:
      ValueError: Raised if index is not a DataIndex object.
    """
    self._task = task if task else (lambda x: x)
    if not isinstance(data_index, self.DataIndex):
      raise ValueError('Expecting `data_index` to be a DataIndex object.')
    self._data_index = data_index
    super(GradebookItem, self).__init__(*args, name=name)

  def FieldIndex(self):
    return '%s.%s.%s' % (self._data_index.message_name,
                         self._data_index.source,
                         self._data_index.field)

  def _RegisterInputs(self):
    return [self._data_index]

  def _Check(self, *args):
    for arg in args:
      if arg is None or (isinstance(arg, numpy.ndarray) and not arg.size):
        return
    self._CheckByRange(self._name, self._task(*args), self._normal_ranges,
                       self._warning_ranges)


class GradebookFilteredItem(base_check.FilteredCheckItem):
  """A filtered checklist item created from the gradebook."""

  @base_check.RegisterSpecs
  def __init__(self, name, data_index, task, *args):
    """Initialize a filtered gradebook item.

    Args:
      name: Name of the check.
      data_index: index to the data source. E.g. MotorStatus.MotorSbi.omega.
      task: A callback function to preprocess the input.
      *args: Please refer to FilteredCheckItem.

    Raises:
      ValueError: Raised if index is not a DataIndex object.
    """
    self._task = task if task else (lambda x: x)
    if not isinstance(data_index, self.DataIndex):
      raise ValueError('Expecting `data_index` to be a DataIndex object.')
    self._data_index = data_index
    super(GradebookFilteredItem, self).__init__(*args, name=name)

  def _RegisterInputs(self):
    return [self._data_index]

  def _Check(self, *args):
    for arg in args:
      if arg is None or (isinstance(arg, numpy.ndarray) and not arg.size):
        return
    self._CheckByRange(self._name, self._task(*args), self._normal_ranges,
                       self._warning_ranges)


class GradebookChecks(base_check.ListOfChecks):
  """A checklist class that is initialized from a gradebook."""

  def Initialize(self, gradebook, for_log, use_full_name=False):
    """Convert a gradebook into a checklist."""
    for message_name, sources in gradebook.GetFieldMap('').iteritems():
      for source, attributes in sources.iteritems():
        # The gradebook is organized similarly to the AIO message structure.
        gradebook_prefix = base_check.BaseCheckItem.AIO_TEMPLATE.substitute(
            message_type=message_name, aio_node=source)

        for fields in attributes.values():
          for field in fields:
            reference = gradebook.GetCriteria(
                gradebook_prefix.split('.') + field.split('.'))
            field_checklist = GradebookChecks()
            field_checklist.InitializeByField(
                for_log, message_name, source, field, reference, use_full_name)
            self.Concatenate(field_checklist)

  def _GetCallback(self, reference):
    """Construct a single argument function to preprocess the values."""
    def ListToArray(x):
      return numpy.array(x) if isinstance(x, list) else x

    def ApplyCallback(callback, args, x):
      """Apply the callback function to the value(s)."""
      if isinstance(x, numpy.ndarray):
        return numpy.array([callback(v, *args) for v in x.flatten()]).reshape(
            x.shape)
      else:
        return callback(x, *args)

    if reference.callback:
      args = reference.callback_args if reference.callback_args else []
      return lambda x: ApplyCallback(reference.callback, args, ListToArray(x))
    else:
      return ListToArray

  def _GetItemName(self, category, reference, data_index, use_full_name):
    """Get the name of the GradebookItem.

    Args:
      category: A category telling what this item checks.
      reference: A Gradebook.LeafNode object.
      data_index: A BaseCheckItem.DataIndex object.
      use_full_name: True if the full name of the check is desired.

    Returns:
      The chosen name of the GradebookItem.
    """

    if use_full_name:
      field_name = reference.name if reference.name else data_index.field
      return '%s.%s (%s)' % (data_index.source, field_name, category)
    else:
      return category

  def _GetValueCallback(self, for_log, preprocess):
    return (preprocess if for_log
            else lambda x: numpy_utils.NumpyToJson(preprocess(x)))

  def _GetNormCallback(self, for_log, preprocess):
    if for_log:
      return lambda x: numpy.linalg.norm(preprocess(x), axis=1)
    else:
      return lambda x: numpy_utils.NumpyToJson(  # pylint: disable=g-long-lambda
          numpy.linalg.norm(preprocess(x), axis=0))

  def _GetStdDevCallback(self, for_log, preprocess):
    if for_log:
      return lambda x: numpy.std(preprocess(x), axis=1)
    else:
      return lambda x: numpy_utils.NumpyToJson(numpy.std(preprocess(x), axis=0))

  def InitializeByField(self, for_log, message_name, source, field, reference,
                        use_full_name):
    """Convert a gradebook item into checklist items.

    Args:
      for_log: True if this checklist item checks log data. False if it checks
          AIO messages.
      message_name: Name of the message.
      source: Name of the AIO node.
      field: The index path within a message. E.g., 'fc_mon.raw.analog[0]'.
      reference: A Gradebook.LeafNode object.
      use_full_name: True if the items should have the full name.
    """

    data_index = base_check.BaseCheckItem.DataIndex(message_name, source, field)
    # The preprocess callback converts units into human readable format.
    preprocess = self._GetCallback(reference)
    if reference.normal_ranges or reference.warning_ranges:
      # Check individual values.
      self._items_to_check.append(GradebookItem(
          self._GetItemName('Value', reference, data_index, use_full_name),
          data_index, self._GetValueCallback(for_log, preprocess), for_log,
          reference.normal_ranges, reference.warning_ranges))

    if reference.normal_norm_ranges or reference.warning_norm_ranges:
      # The field should be a vector, and we check its norm.
      name = self._GetItemName(
          'Norm', reference, data_index, use_full_name)
      self._items_to_check.append(GradebookItem(
          name, data_index, self._GetNormCallback(for_log, preprocess), for_log,
          reference.normal_norm_ranges, reference.warning_norm_ranges))

    if reference.normal_std_dev_ranges or reference.warning_std_dev_ranges:
      # Check the standard deviation of the values.
      name = self._GetItemName(
          'Std. Dev.', reference, data_index, use_full_name)
      self._items_to_check.append(GradebookFilteredItem(
          name, data_index, self._GetStdDevCallback(for_log, preprocess),
          for_log, GRADEBOOK_FILTER_WINDOW_SIZE, GRADEBOOK_FILTER_WINDOW_STEP,
          reference.normal_std_dev_ranges, reference.warning_std_dev_ranges))
