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

"""Utilities to parse gradebook.json and manage criteria to auto-check."""

import collections
import importlib
import re
import string

from makani.analysis.checks import check_range
from makani.avionics.network import network_config
from makani.lib.python import dict_util


class GradebookParserError(Exception):
  pass


class Gradebook(object):
  """The class to store the list of checks to perform."""

  LeafNode = collections.namedtuple(  # pylint: disable=invalid-name
      'LeafNode', ['normal_ranges', 'warning_ranges',
                   'normal_std_dev_ranges', 'warning_std_dev_ranges',
                   'normal_norm_ranges', 'warning_norm_ranges',
                   'callback', 'callback_args', 'name'])

  def __init__(self, definitions, network_yaml_file):
    """Loads the gradebook as a JSON object.

    Args:
      definitions: A dictionary of check definitions formated as:
          {
              'imports': {<short_name>: <path_to_the_module>},  // Optional
              'checks': {  // Required.
                  <message_type> : {
                      // This can be a name or a regex. The regex should be
                      // wrapped by parentheses.
                      //
                      <aio_node>: {
                          // The nest struct reflects the C message.
                          // Indices can apply if the attribute is a dict/list.
                          // Example: array[:], array[1], dict[:], dict[key]
                          //
                          <attribute>: {
                              <sub_attribute>: {
                                  // Ranges are list of pairs or scalars, where
                                  // a pair denotes lower and upper bounds, and
                                  // a scalar denotes an exact match.
                                  // Ranges can also be "any", meaning
                                  // any value falls into the range.
                                  // Ranges can also be an empty list, meaning
                                  // no value falls into the range.
                                  //
                                  "normal_ranges": [],
                                  "warning_ranges": [],
                                  "normal_std_dev_ranges": [],
                                  "warning_std_dev_ranges": [],
                                  "normal_norm_ranges": [],
                                  "warning_norm_ranges": [],
                                  // Callback to preprocess the data.
                                  "callback": "<module_short_name>.<function>",
                                  "callback_args": [<constant_values>],
                                  "name": Name of the field,
                              }
                          }
                      }
                  }
              }
          }
      network_yaml_file: The YAML file that defines nodes and messages on the
          AIO network.

    Raises:
      GradebookParserError: Raised if the gradebook is invalid.
    """

    self._imports = self._ImportModules(definitions.get('imports', {}))

    self._checks = {}
    message_types = network_config.NetworkConfig(
        network_yaml_file).all_messages
    sources_by_message_type = {m.name: {s.camel_name for s in m.all_senders}
                               for m in message_types}

    if 'checks' not in definitions:
      raise GradebookParserError(
          'Found no checks in gradebook.')

    for message_type, aio_nodes in definitions['checks'].iteritems():
      if message_type not in sources_by_message_type:
        raise GradebookParserError(
            'Invalid message type "%s" in gradebook.' % message_type)
      self._checks[message_type] = {}
      # Use regular expression to pick AIO nodes to check.
      # wildcard_info[regex] = checks_to_perform
      wildcard_info = {}
      for aio_node, info in aio_nodes.iteritems():
        # Names wrapped in parentheses are wildcards for batch-selection.
        if aio_node.startswith('(') and aio_node.endswith(')'):
          wildcard_info[aio_node[1:-1]] = info
        else:
          self._checks[message_type][aio_node] = self._InitializeNodes(info)

      # Expand regular expressions such as '.*' if there is any.
      for pattern, info in wildcard_info.iteritems():
        regex = re.compile(pattern)
        for aio_node in sources_by_message_type[message_type]:
          if regex.match(aio_node):
            if aio_node not in self._checks[message_type]:
              self._checks[message_type][aio_node] = (
                  self._InitializeNodes(info))
            else:
              raise GradebookParserError(
                  'Regex %s matches previously added node %s for '
                  'message type %s.' % (pattern, aio_node, message_type))

  def _ValidateLeafNode(self, node):
    """Validate a Gradebook node."""
    def ValidateRangePair(normal_ranges, warning_ranges):
      if normal_ranges or warning_ranges:
        if not normal_ranges:
          raise GradebookParserError(
              'Need at least `normal_ranges` to make a range pair.')

    ValidateRangePair(node.normal_ranges, node.warning_ranges)
    ValidateRangePair(node.normal_norm_ranges, node.warning_norm_ranges)
    ValidateRangePair(node.normal_std_dev_ranges, node.warning_std_dev_ranges)

  def _InitializeNodes(self, info):
    """Initializes a leaf node as a struct with a set of criteria."""
    def GetWarningChecker(mapping, warning_check_type, normal_check_type):
      normal_ranges = mapping.get(normal_check_type, None)
      default = 'any' if normal_ranges else None
      return GetChecker(mapping, warning_check_type, default)

    def GetChecker(mapping, check_type, default):
      value = mapping.get(check_type, default)
      if value == 'any':
        return check_range.AllInclusiveRange()
      elif value is None:
        # No checks for this type.
        return None
      else:
        return check_range.RangeChecker(value)

    results = {}
    if self._IsLeafNode(info):
      leaf_node = self.LeafNode(
          # Leaving default as None means no checks are available.
          GetChecker(info, 'normal_ranges', None),
          GetWarningChecker(info, 'warning_ranges', 'normal_ranges'),
          GetChecker(info, 'normal_std_dev_ranges', None),
          GetWarningChecker(info, 'warning_std_dev_ranges',
                            'normal_std_dev_ranges'),
          GetChecker(info, 'normal_norm_ranges', None),
          GetWarningChecker(info, 'warning_norm_ranges', 'normal_norm_ranges'),
          self._GetCallback(info.get('callback', None)),
          info.get('callback_args', []),
          info.get('name', None),
      )
      self._ValidateLeafNode(leaf_node)
      return leaf_node
    else:
      for key, value in info.iteritems():
        key = self._ParseKey(key)
        if key:
          results[key] = self._InitializeNodes(value)
    return results

  def _ParseKey(self, key):
    """Parse gradebook keys to field names.

    Keys may have array indices referring to enums in the format of
    'module.enum_prefix.short_name'. This function converts the enum string
    into the enum value. Example: 'omega[aio_labels.kMotorSbo] => omega[0]'.

    Args:
      key: A key in the gradebook.

    Returns:
      The key with parsed array indices.

    Raises:
      GradebookParserError: Raised if the key is invalid.
    """

    new_key = ''
    while '[' in key:
      left_pos = key.find('[')
      new_key += key[:left_pos]
      right_pos = key.find(']', left_pos)
      index = key[left_pos + 1 : right_pos]
      key = key[right_pos + 1:]
      if '.' in index:
        split_pos = index.find('.')
        module_label = index[:split_pos]
        full_name = index[split_pos + 1:]
        if not hasattr(self._imports[module_label], full_name):
          raise GradebookParserError(
              '%s does not exist in module %s.' % (full_name, module_label))
        else:
          index = str(getattr(self._imports[module_label], full_name))
      new_key += '[%s]' % index
    if key:
      new_key += key
    return new_key

  def _IsLeafNode(self, node):
    """Returns True if `node` is not a field but a collection of criteria."""
    # Right now, we judge by checking if there are any fields with dict values.
    if isinstance(node, dict):
      return dict not in {type(v) for v in node.itervalues()}
    return False

  def _ImportModules(self, module_names):
    """Imports modules referenced in the "imports" section of the gradebook."""
    imported_modules = {}
    for name, path in module_names.iteritems():
      try:
        imported_modules[name] = importlib.import_module(path)
      except ImportError:
        raise GradebookParserError(
            'Cannot import module "%s" from "%s".' % (name, path))
    return imported_modules

  def _GetCallback(self, name):
    """Gets the callback function to post-process the raw values."""
    if not name:
      return None
    items = name.rsplit('.', 1)
    if len(items) != 2:
      raise GradebookParserError(
          'Expecting a callback name in the format of '
          '"<module>.<function>", got "%s".' % (name))

    module_label, function = items
    if module_label not in self._imports:
      raise GradebookParserError(
          'Cannot find "%s" in the "imports".' % module_label)

    if not hasattr(self._imports[module_label], function):
      raise GradebookParserError(
          'Cannot find function "%s" in module "%s".' % (function,
                                                         module_label))

    return getattr(self._imports[module_label], function)

  def GetCriteria(self, indices):
    """Gets the criteria struct using a list of indices."""
    return dict_util.GetByPath(self._checks, indices)

  def GetFieldMap(self, template):
    return {message_type: self.GetFieldMapByMessageType(message_type, template)
            for message_type in self._checks}

  def GetFieldMapByMessageType(self, message_type, template):
    """Gets indices of fields, grouped by source and first-level attributes.

    Source is the AIO node where a message comes from.

    Args:
      message_type: Name of the message type.
      template: The string template to use as a prefix of the field indices.

    Returns:
      A nested dictionary of attributes, where indices for each field is first
      grouped by first-level attributes of a message type, then by the source.
      I.e., attributes[source][attribute_name] = [field_path]

    Example:
      {'MessageType': {'AioNode': {'a': {'b': {'c': 0, 'd': 1}}}}}
      is turned into
      {'AioNode': {'a': [PREFIX + 'a.b.c', PREFIX + 'a.b.d']}}, where
      PREFIX is `template` with message_type and aio_node substituted.
    """

    def _ListFieldPaths(gradebook, fields, prefix):
      """Returns paths to all fields in the `fields` dictionary."""
      if isinstance(fields, gradebook.LeafNode):
        return [prefix]
      field_paths = []
      for name, children in fields.iteritems():
        field_paths += _ListFieldPaths(gradebook, children, prefix + '.' + name)
      return field_paths

    if message_type not in self._checks:
      return {}
    message_gradebook = self._checks[message_type]

    attributes_per_source = {}
    header = string.Template(template)
    for source, attributes in message_gradebook.iteritems():
      # `attributes` is a nested dictionary structured similar to AIO message,
      # except that the leaves are not values, but dictionaries of criteria.
      prefix = header.substitute(message_type=message_type, aio_node=source)
      field_paths = {}
      for name, fields in attributes.iteritems():
        field_paths[name] = _ListFieldPaths(self, fields, prefix + name)
      attributes_per_source[source] = field_paths
    return attributes_per_source
