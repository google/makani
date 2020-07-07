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

"""View functions to interact with web clients."""

import atexit
import json
import logging
import os
import re
import string
import time

from django import http
from django import shortcuts
from django import template
from django.core import urlresolvers

from makani.analysis.checks import log_util
from makani.avionics.network import message_type as aio_message_type
from makani.avionics.network import network_config
from makani.gs.monitor2.apps.layout import autogen
from makani.gs.monitor2.apps.layout import base as layout_base
from makani.gs.monitor2.apps.layout import layout_util
from makani.gs.monitor2.apps.layout import loader
from makani.gs.monitor2.apps.layout import memory as layout_memory
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.layout import widgets
from makani.gs.monitor2.apps.receiver import receiver_manager
from makani.gs.monitor2.apps.receiver import views as receiver_views
from makani.gs.monitor2.project import settings
from makani.lib.bazel import bazel_util
from makani.lib.python import c_helpers
from makani.lib.python import debug_util
from makani.lib.python import struct_tree
from makani.lib.python.h5_utils import h5_io
import numpy

MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', aio_message_type)

CONFIG_FILES = {
    'plot_defs': os.path.join(settings.MONITOR_PATH, 'configs/plot_defs.json'),
}


def Home(request):
  """Get the response for the home page."""

  layout_names = loader.LayoutLoader().Names()
  layout_names.sort()
  all_layouts = [
      {'name': layout,
       'url': urlresolvers.reverse(
           'view_aio_layout', args=[loader.LayoutLoader().ModuleName(layout)])}
      for layout in layout_names]
  context = {
      'layouts': all_layouts,
      'canvas_cols': settings.CSS_GRID_COLUMNS,
  }
  _CreateAndAddClientIdToContext(context)

  template_name = 'home.html'
  return shortcuts.render(request, template_name, context,
                          context_instance=template.RequestContext(request))


def _ListFiles(path_arg):
  """List files under a local path."""
  path_template = string.Template(path_arg)
  prefix_path = path_template.substitute(os.environ)
  sub_paths = os.listdir(prefix_path)
  return prefix_path, sub_paths


def _GetFullFilePath(prefix_path, sub_path):
  return os.path.join(prefix_path, sub_path)


def SelectAllLogs(request):
  """Select all logs in the last visited directory."""
  current_path = request.session['current_path']
  try:
    prefix_path, sub_paths = _ListFiles(current_path)
  except OSError:
    return http.HttpResponse('Cannot list directory "%s"!' % current_path)

  file_list = []
  for sub_path in sorted(sub_paths):
    # Construct the full path.
    if sub_path.endswith('.h5') and not sub_path.startswith('format'):
      full_path = _GetFullFilePath(prefix_path, sub_path)
      if not os.path.isdir(full_path):
        file_list.append(full_path)
  return http.HttpResponse(';\n'.join(file_list))


def Console(request, command, args):
  """Take commandlines from the client and respond with console outputs.

  Args:
    request: The HTML resquest object.
    command: The command to be run. Only 'ls' is permitted for now.
    args: The string of arguments to the command.

  Returns:
    The HttpResponse telling the output of the command.
  """

  if command != 'ls':
    message = 'Command "%s" is not allowed.' % command
    return http.HttpResponse(message)

  arg_template = string.Template(args)
  arg_path = arg_template.safe_substitute(
      {'MAKANI_HOME': bazel_util.GetWorkspaceRoot()})

  try:
    prefix_path, sub_paths = _ListFiles(arg_path)
    request.session['current_path'] = arg_path
  except OSError:
    return http.HttpResponse('Cannot list directory "%s"!' % arg_path)

  file_list = []
  for sub_path in sorted(sub_paths):
    # Construct the full path.
    full_path = _GetFullFilePath(prefix_path, sub_path)
    if os.path.isdir(full_path):
      # If this is a directory, add the javascript to allow users to click
      # into it.
      file_list.append(
          '<a href="javascript:void(0)" onclick="onListFiles(\'%s\')">%s</a>'
          % (full_path, sub_path))
    elif sub_path.endswith('.h5') and not sub_path.startswith('format'):
      # If this is an HDF5 file, add the javascript to allow users to
      # visualize it.
      file_list.append(
          '<a href="javascript:void(0)" onclick="onAddLog(\'%s\')">%s</a>'
          % (full_path, sub_path))
    else:
      file_list.append(sub_path)
  text = '<br>'.join(file_list)
  return http.HttpResponse(text)


def _GetMinMessageFrequency():
  """Get the minimum frequency across all message types."""
  config = network_config.NetworkConfig(settings.NETWORK_YAML)
  return min(m.frequency_hz for m in config.all_messages if m.frequency_hz > 0)


def _TryToEnforceAioReceiver(client_id):
  """Ensure that the client is subscribed to the AioReceiver."""
  # TODO: Investigate always running the AioReceiver.
  message_receiver = receiver_manager.ReceiverManager.GetReceiver(client_id)
  if not message_receiver:
    if receiver_manager.ReceiverManager.CheckAndStartAioReceiver(
        client_id, receiver_views.CreateAioReceiver):
      # A new AioReceiver is started.
      # Get the longest period for all messages, and multiply it by two to
      # make sure we do not miss any message.
      time.sleep(2.0 / _GetMinMessageFrequency())
    return receiver_manager.ReceiverManager.GetReceiver(client_id)
  else:
    return message_receiver


def ViewMessageType(request, client_id, message_type,
                    template_name='monitor.html'):
  """View information within a message by automatically generating a layout.

  Args:
    request: An HttpRequest from the client.
    client_id: The ID of the client's browser tab.
    message_type: The Enum name of a message type.
    template_name: The HTML template used to render the layout.

  Returns:
    An HttpResponse in the format of a serialized JSON object.
  """
  configs = _LoadConfigs()
  _TryToEnforceAioReceiver(client_id)
  resp = _GetMessage(request, client_id, message_type)
  resp = resp.Data(convert_to_basic_types=True) if resp else {}

  configs['scenarios'] = autogen.GenerateScenario(resp, message_type)
  context = _PrepareContext(configs)
  new_client_id = _CreateAndAddClientIdToContext(context)
  context['periodic_url'] = '/dashboard/periodic/msg_enum/%s/%s' % (
      new_client_id, message_type)
  context['content_width'] = settings.CSS_GRID_COLUMNS
  context['order_horizontally'] = True
  return shortcuts.render(request, template_name, context,
                          context_instance=template.RequestContext(request))


def UpdateMessageOptions(unused_request, client_id):
  """Detect what messages have been received and update the client.

  Args:
    unused_request: An HttpRequest from the client.
    client_id: The ID of the client's browser tab.

  Returns:
    An HttpResponse about a dictionary of {message_enum: message_short_name}
  """
  message_receiver = _TryToEnforceAioReceiver(client_id)
  info = message_receiver.GetReceivedMessageTypes() if message_receiver else []
  return http.HttpResponse(json.dumps(info))


def ViewAioLayout(request, layout_name):
  """Open a monitor layout that get data from AIO.

  Args:
    request: An HttpRequest from the client.
    layout_name: Name of the layout associated with the client.

  Returns:
    An HttpResponse in the format of a serialized JSON object.
  """
  context = {'receiver_type': 'aio'}
  return _ViewLayout(request, layout_name, context)


def BrowseLog(request, path):
  """Browse the log by expanding the field at `path`.

  Args:
    request: An HttpRequest from the client.
    path: A path pointing to one field in the log.

  Returns:
    An HttpResponse serializing a list of names for child fields.
  """

  # The log structure may differ across logs, we always use the first log to
  # construct the log structure.
  log_path = request.session['log_paths'][0]
  log_data = struct_tree.StructTree(log_path, fail_silently=True, readonly=True)
  try:
    skeleton = log_data.Skeleton(path, depth=1)
  except h5_io.H5IndexError:
    return http.HttpResponse('{}')
  parent_path = path
  d3_data = struct_tree.DictToD3Tree(skeleton, '.', parent_path)
  if 'children' in d3_data:
    # The first layer is a placeholder. Starts from the second layer.
    return http.HttpResponse(json.dumps(d3_data['children']))
  else:
    return http.HttpResponse('{}')


def ViewLogStructure(request, paths, template_name='log_structure.html'):
  """View structure of an HDF5 log at given log path.

  Args:
    request: An HttpRequest from the client.
    paths: Paths to the local log files.
    template_name: The HTML template used to render the layout.

  Returns:
    An HttpResponse that renders the log structure.
  """

  # `context` includes variables used to render the HTML.
  context = {
      'graph_width': 6000,
      'graph_height': 6000,
      'frame_width': 200,
      'frame_height': 540,
      'canvas_cols': 12,
  }

  log_paths = []
  for path in paths.split(';'):
    path = path.strip()
    if not path:
      continue
    path_template = string.Template(path)
    log_path = path_template.substitute(os.environ)
    basename = os.path.basename(log_path)
    if basename.startswith('(') and basename.endswith(')'):
      dirname = os.path.dirname(log_path)
      regex_pattern = re.compile(basename[1:-1]+'$')
      filenames = os.listdir(dirname)
      matched_files = [f for f in filenames if regex_pattern.match(f)]
      log_paths += [os.path.join(dirname, f) for f in matched_files]
    else:
      log_paths.append(log_path)

  if not log_paths:
    context['errors'] = 'Cannot find log data'
  else:
    # Use the first log to index fields.
    log_data = struct_tree.StructTree(
        log_paths[0], fail_silently=True, readonly=True)
    log_skeleton = log_data.Skeleton(depth=1)
    d3_data = struct_tree.DictToD3Tree(log_skeleton, '/')
    d3_data['expand_url'] = urlresolvers.reverse('browse_log', args=[''])
    request.session['log_paths'] = log_paths
    context['skeleton'] = json.dumps(d3_data)

  order_horizontally = True
  configs = _LoadConfigs()
  scenarios = layout_base.AssembleLayout([
      ('Signals', [
          widgets.DictLinesWidget('series', None, interactive=True,
                                  use_markers=True),
      ]),
  ], desired_view_cols=1, order_horizontally=order_horizontally)
  layout_names = loader.LayoutLoader().ModuleNames()
  layout_names.sort()
  configs['scenarios'] = scenarios
  context.update(_PrepareContext(configs))
  context['layout_names'] = layout_names
  context['content_width'] = settings.CSS_GRID_COLUMNS - 2
  context['order_horizontally'] = order_horizontally
  _CreateAndAddClientIdToContext(context)
  return shortcuts.render(request, template_name, context,
                          context_instance=template.RequestContext(request))


def PeriodicDataPoll(request, client_id, layout_name):
  """Compute realtime data and respond to periodic polling from a client layout.

  Args:
    request: An HttpRequest from the client.
    client_id: The ID of the client's browser tab.
    layout_name: Name of the layout associated with the client.

  Returns:
    An HttpResponse in the format of a serialized JSON object.
  """
  aggregated_message = _GetMessage(request, client_id)
  if not aggregated_message:
    aggregated_message = struct_tree.StructTree(
        {}, fail_silently=True, readonly=True)
  layout = loader.LayoutLoader().GetLayoutByModuleName(layout_name)
  tab_memory = layout_memory.GetMemory(client_id, False)
  if tab_memory is not None:
    # Load the persistent memory.
    layout.Import(tab_memory)
  else:
    layout.Initialize()
    tab_memory = layout_memory.GetMemory(client_id, True)

  # Start the AIO receiver in case the server has restarted.
  _TryToEnforceAioReceiver(client_id)

  try:
    data = layout.Filter(aggregated_message)
  except Exception:  # pylint: disable=broad-except
    # layout.Filter may introduce any kind of exception.
    logging.error('PeriodicDataPoll encountered an error:\n%s',
                  debug_util.FormatTraceback())

    layout.Export(tab_memory)
    return http.HttpResponse('{}')

  # Save the persistent memory.
  layout.Export(tab_memory)
  resp = data.Json()
  if settings.DEBUG:
    resp['__message__'] = '\n-----------------------------\n'.join(
        'Error in indicator "%s":\n%s' % (k, v)
        for k, v in layout.ErrorReport())
  resp_str = json.dumps(resp)
  layout.ClearErrors()
  return http.HttpResponse(resp_str)


def _DownSample(data, length):
  window_size = max(1, len(data)/length)
  if window_size > 1:
    data = data[:len(data) / window_size * window_size]
    return numpy.mean(data.reshape(-1, window_size), 1), window_size
  else:
    return data, 1


def GetLogData(request, mode, fields):
  """Get values of data fields within a log file."""

  log_paths = request.session['log_paths']
  fields = [f.strip() for f in fields.split('\n') if f.strip()]
  field_labels = layout_util.GetDistinguishableNames(
      fields, '.', ['kAioNode', 'kMessageType'])

  if mode == 'merge':
    series = ConcatenateLogData(log_paths, field_labels)
  else:  # By default, mode = 'compare'
    series = CompareLogData(log_paths, field_labels)
  resp = {'series': series}
  return http.HttpResponse(json.dumps(resp))


def _StringReplace(subject, translate):
  for s, t in translate:
    subject = subject.replace(s, t)
  return subject


def GetMessageSnapshot(request, client_id, title):
  aggregated_message = _GetMessage(request, client_id)
  result = aggregated_message.Data(True)
  response = http.HttpResponse(content_type='text/plain')
  response['Content-Disposition'] = (
      'attachment; filename=snapshot_%s.json' % title)
  response.write(json.dumps(result, indent=2))
  return response


def GetRawLogData(request, fields):
  """Get values of data fields within a log file."""
  log_paths = request.session['log_paths']
  fields = [f.strip() for f in fields.split('\n') if f.strip()]
  field_labels = layout_util.GetDistinguishableNames(
      fields, '.', ['kAioNode', 'kMessageType'])

  result = {}
  # Remove special characters so variables can be parsed and loaded into Matlab.
  bad_chars = ['.', ',', '-', '+', '(', ')', '[', ']', '{', '}', ':',
               'kMessageType', 'kAioNode', 'messages', 'message']
  replacement = list(zip(bad_chars, ['_'] * len(bad_chars)))
  replacement = [('[:]', ''), (':,', ''), (' ', '')] + replacement
  for log_path in log_paths:
    base_name = os.path.basename(log_path)
    log_name = 'log_' + _StringReplace(base_name[:base_name.find('.')],
                                       replacement)
    log_data = struct_tree.StructTree(
        log_path, fail_silently=True, readonly=True)
    result[log_name] = {}
    for field, legend_label in field_labels.iteritems():
      data, timestamps = log_util.GetOrderedDedupDataAndTimeByField(
          log_data, field, rebase=False)
      result[log_name][_StringReplace(legend_label, replacement)] = {
          'values': data.tolist() if data is not None else None,
          'timestamps': timestamps.tolist() if timestamps is not None else None,
          'status': 'success' if data is not None else 'missing',
      }

  response = http.HttpResponse(content_type='text/plain')
  response['Content-Disposition'] = 'attachment; filename=makani_log_data.json'
  response.write(json.dumps(result, indent=2))
  return response


def ConcatenateLogData(log_paths, field_labels):
  """Get series of data, each corresponding to field values in all logs."""
  series = {}
  base_timeline = float('inf')
  for log_path in log_paths:
    log_data = struct_tree.StructTree(
        log_path, fail_silently=True, readonly=True)
    for field, legend_label in field_labels.iteritems():
      data, timestamps = log_util.GetOrderedDedupDataAndTimeByField(
          log_data, field, rebase=False)
      if data is None or timestamps is None:
        continue
      base_timeline = min(base_timeline, float(timestamps[0]))
      if legend_label not in series:
        series[legend_label] = {'x': timestamps, 'y': data}
      else:
        series[legend_label]['x'] = numpy.concatenate(
            (series[legend_label]['x'], timestamps))
        series[legend_label]['y'] = numpy.concatenate(
            (series[legend_label]['y'], data))

  result = {}
  for field, legend_label in field_labels.iteritems():
    timestamps, _ = _DownSample(
        series[legend_label]['x'], settings.MAX_DATA_POINTS_PER_LOG_FIELD)
    data, downsample_rate = _DownSample(
        series[legend_label]['y'], settings.MAX_DATA_POINTS_PER_LOG_FIELD)
    if downsample_rate > 1:
      legend_label += '(/%d)' % downsample_rate
    result[legend_label] = {'x': (timestamps - base_timeline).tolist(),
                            'y': data.tolist()}
  return result


def CompareLogData(log_paths, field_labels):
  """Get series of data, each corresponding to field values within a log."""
  series = {}
  base_timeline = float('inf')
  for log_path in log_paths:
    log_data = struct_tree.StructTree(
        log_path, fail_silently=True, readonly=True)
    log_name = os.path.basename(log_path)
    if '.' in log_name:
      log_name = log_name[:log_name.rfind('.')]
    for field, legend_label in field_labels.iteritems():
      data, timestamps = log_util.GetOrderedDedupDataAndTimeByField(
          log_data, field, rebase=True)
      if data is None or timestamps is None:
        continue
      data, _ = _DownSample(data, settings.MAX_DATA_POINTS_PER_LOG_FIELD)
      timestamps, downsample_rate = _DownSample(
          timestamps, settings.MAX_DATA_POINTS_PER_LOG_FIELD)
      base_timeline = min(base_timeline, float(timestamps[0]))
      short_name = '%s.%s' % (log_name, legend_label)
      if downsample_rate > 1:
        short_name += '(/%d)' % downsample_rate
      series[short_name] = {'x': timestamps,
                            'y': data.tolist()}
  for short_name in series:
    series[short_name]['x'] = (series[short_name]['x'] - base_timeline).tolist()
  return series


def PeriodicMessagePoll(request, client_id, message_type=None):
  """Retrieve realtime data and respond to periodic polling from a message view.

  Args:
    request: An HttpRequest from the client.
    client_id: The ID of the client's browser tab.
    message_type: The Enum name of a message type.

  Returns:
    An HttpResponse in the format of a serialized JSON object.
  """
  resp = _GetMessage(request, client_id, message_type)
  if not resp:
    resp = {}
  else:
    resp = resp.Data(convert_to_basic_types=True)
  resp_str = json.dumps(resp)
  return http.HttpResponse(resp_str)


def _LoadConfigs():
  """Load default layout configuration parameters."""
  configs = {}
  for cf, filename in CONFIG_FILES.iteritems():
    with open(filename, 'r') as fp:
      configs[cf] = json.load(fp)
  if 'plot_defs' not in configs:
    logging.Error('Missing definitions for plotting javascripts.')
  return configs


def _PrepareContext(configs):
  """Prepare the context to render the layout."""
  context = {}
  fig_templates = set()

  canvas_cols = configs['scenarios']['canvas']['grid_width']
  context['canvas_cols'] = canvas_cols
  row_height_px = configs['scenarios']['canvas']['row_height_px']

  ui_objs = []
  max_cols = canvas_cols
  for stripe in configs['scenarios']['views']:
    for view in stripe['stripe']:
      view['canvas_cols'] = int(
          float(view['grid_width']) / stripe['grid_width'] * canvas_cols + 0.5)
      for indicator in view['indicators']:
        ui_obj = indicator
        if 'rows' not in ui_obj:
          ui_obj['height'] = 'auto'
        else:
          rows = ui_obj['rows']
          ui_obj['height'] = str(rows * row_height_px) + 'px'
        if 'cols' not in ui_obj:
          ui_obj['cols'] = max_cols
        # TODO: Change `id` to 'indicator_id', and 'selector'
        # to 'dom_selector'.
        ui_obj['id'] = 'ui_obj_%s' % len(ui_objs)
        ui_obj['selector'] = '#%s' % (ui_obj['id'])

        ui_objs.append(ui_obj)
        fig_templates.add(ui_obj['template'])

  context['fig_templates'] = fig_templates
  context['plot_defs'] = configs['plot_defs']
  context['views'] = configs['scenarios']['views']
  context['ui_objs_str'] = json.dumps(ui_objs)
  context['stoplight_error'] = stoplights.STOPLIGHT_ERROR
  context['stoplight_warning'] = stoplights.STOPLIGHT_WARNING
  context['stoplight_normal'] = stoplights.STOPLIGHT_NORMAL
  context['stoplight_unavailable'] = stoplights.STOPLIGHT_UNAVAILABLE
  context['stoplight_any'] = stoplights.STOPLIGHT_ANY
  return context


def _GetMessage(unused_request, client_id, message_type=None):
  """Get a message from the receiver."""
  message_receiver = receiver_manager.ReceiverManager.GetReceiver(client_id)
  resp = struct_tree.StructTree({}, fail_silently=True, readonly=True)
  if message_receiver:
    if message_type is not None:
      message_enum = MESSAGE_TYPE_HELPER.Value(message_type)
    else:
      message_enum = None
    resp = message_receiver.GetLatest(message_enum)
  return resp


def _CreateAndAddClientIdToContext(context):
  client_id = receiver_manager.ReceiverManager.GetNewClientId()
  context['client_id'] = client_id
  return client_id


def _ViewLayout(request, layout_name, extra_context=None):
  """Get a monitor layout according to `layout_name`."""
  layout = loader.LayoutLoader().GetLayoutByModuleName(layout_name)
  if layout is None:
    return http.HttpResponseRedirect(urlresolvers.reverse('home'))
  layout.Initialize()
  configs = _LoadConfigs()
  configs['scenarios'] = layout.Scenario()
  context = _PrepareContext(configs)
  client_id = _CreateAndAddClientIdToContext(context)
  # Initialize the layout.
  layout.Export(layout_memory.GetMemory(client_id, True))
  # Add polling URL.
  context['periodic_url'] = '/dashboard/periodic/layout/%s/%s' % (client_id,
                                                                  layout_name)
  context['layout_name'] = layout_name
  context['content_width'] = settings.CSS_GRID_COLUMNS
  context['order_horizontally'] = layout.OrderHorizontally()
  context['default_font_size'] = layout.DefaultFontSize()
  context['sim_mode'] = settings.POPULATE_MESSAGES_FROM_SIM

  if extra_context:
    context.update(extra_context)

  template_name = 'monitor.html'
  return shortcuts.render(request, template_name, context,
                          context_instance=template.RequestContext(request))
