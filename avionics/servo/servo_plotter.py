#!/usr/bin/python
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


"""A script for plotting various data sets from the servo controllers."""

import collections
import ctypes
import importlib
import signal
import socket
import sys
import time

from makani.avionics.common import aio
from makani.avionics.common import pack_avionics_messages as avionics_messages
from makani.avionics.common import servo_types
from makani.avionics.firmware.monitors import servo_types as servo_monitor_types
from makani.avionics.network import aio_node
from makani.avionics.servo.firmware import r22_types
from makani.lib.python import c_helpers
from makani.lib.python import ctype_util
import numpy
from PySide import QtCore
from PySide import QtGui

aio_node_helper = c_helpers.EnumHelper('AioNode', aio_node)
servo_status_bits = c_helpers.EnumHelper('ServoStatusFlag', servo_types,
                                         'kServoStatus')
servo_warning_bits = c_helpers.EnumHelper('ServoWarningFlag', servo_types,
                                          'kServoWarning')
servo_error_bits = c_helpers.EnumHelper('ServoErrorFlag', servo_types,
                                        'kServoError')

r22_status_bits = c_helpers.EnumHelper('R22StatusBit', r22_types)

# Must be included after PySide in order to force pyqtgraph to use it.
pyqtgraph = importlib.import_module('pyqtgraph')
dockarea = importlib.import_module('pyqtgraph.dockarea')

Alias = collections.namedtuple('Alias', ['base_name'])
Operation = collections.namedtuple('Operation', ['function', 'dtype'])


def MultiplyOp(param, factor):
  return Operation(lambda c, s, d: factor * d[s][param][c], numpy.float64)


def CountsToDegreesOp(param, counts):
  return Operation(
      lambda c, s, d: numpy.mod(d[s][param][c] * 360.0 / counts, 360.0) - 180.0,
      numpy.float64)


def RadiansToDegreesOp(param):
  return MultiplyOp(param, 180.0 / numpy.pi)


def DeltaFirstSourceOp(param):
  return Operation(lambda c, s, d: d[s][param][c] - d[0][param][c],
                   numpy.float64)


class PlotDockArea(dockarea.DockArea):
  """Create plot dock area for GUI.

  This class handles all plotting functionality in the main dock area. Add
  the instantiated object to a QtGui layout, and then call the appropriate
  create plots function(s).
  """

  def __init__(self):
    super(PlotDockArea, self).__init__()
    self._lines = {}
    self._bits = {}
    self._plots = []

  def _AddLine(self, node, name, plot):
    """Add a new data signal to plot redraw list."""
    if node not in self._lines:
      self._lines[node] = []
    self._lines[node].append((name, plot))

  def _AddBit(self, node, name, bit, offset, plot):
    """Add a new bit data signal to plot redraw list from a bit field."""
    if node not in self._bits:
      self._bits[node] = []
    self._bits[node].append((name, bit, offset, plot))

  def _GetPlotPen(self, servos, signals):
    """Helper function to generate different pen colors."""
    signals = [i for i, sig in enumerate(signals) if sig]
    pen = {}
    for i, (_, node) in enumerate(servos):
      for j, sig in enumerate(signals):
        pen[(node, sig)] = i * len(signals) + j
    return (pen, len(pen))

  def _NewPlot(self, title):
    """Helper function to generate new plots with default options."""
    dock = dockarea.Dock(name=title)
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock.addWidget(glw)
    p = glw.addPlot(title=title)
    p.showGrid(True, True)
    p.setMouseEnabled(x=False)
    self._plots.append(p)
    return (p, dock)

  def _PlotAngleBias(self, servos):
    """Plot servo angle bias estimates (for paired servos)."""
    return self.CreatePlot(servos, 'Angle bias', 'Bias', 'deg', ['ang_bias'])

  def _PlotVelocity(self, servos):
    """Plot servo velocity measurements."""
    return self.CreatePlot(servos, 'Velocity', 'Velocity', 'deg/s', ['vel_m'])

  def _PlotCurrent(self, servos):
    """Plot servo current measurements."""
    return self.CreatePlot(servos, 'Current', 'Current', 'A',
                           ['cur_m', 'cur_limit', 'cur_nlimit'])

  def _PlotLineVoltage(self, servos):
    """Plot servo line voltage measurements."""
    return self.CreatePlot(servos, 'Line voltage', 'Line voltage', 'V',
                           ['v_lv_in_a', 'v_lv_in_b', 'v_servo'])

  def _PlotAngleError(self, servos):
    """Plot servo angle error measurements (relative to first servo)."""
    return self.CreatePlot(servos[1:],
                           'Angle error (rel {0})'.format(servos[0][0]),
                           'Angle', 'deg', ['ang_err'])

  def _PlotVelocityError(self, servos):
    """Plot servo velocity error measurements (relative to first servo)."""
    return self.CreatePlot(servos[1:],
                           'Velocity error (rel {0})'.format(servos[0][0]),
                           'Velocity', 'deg/s', ['vel_err'])

  def _PlotCurrentError(self, servos):
    """Plot servo current error measurements (relative to first servo)."""
    return self.CreatePlot(servos[1:],
                           'Current error (rel {0})'.format(servos[0][0]),
                           'Current', 'A', ['cur_err'])

  def CreateLogicPlot(self, servos, title, bits):
    """Plot status bits."""
    (p, dock) = self._NewPlot(title)
    p.addLegend()
    p.setLabel('bottom', 'Time')
    p.setLabel('left', 'Status bits')
    p.setYRange(0.0, len(bits))
    p.setMouseEnabled(x=False, y=False)
    (pen, pens) = self._GetPlotPen(servos, [True] * len(bits))
    for name, node in servos:
      for idx, bitinfo in enumerate(bits):
        param, enum, shortname = bitinfo
        value = enum.Value(shortname)
        # Determine which bit is set.
        bit = bin(value)[::-1].index('1')
        bit_name = name + ' ' + shortname
        self._AddBit(node, param, bit, len(bits) - idx - 1,
                     p.plot(name=bit_name, pen=(pen[(node, idx)], pens)))
    return dock

  def CreatePlot(self, actuators, title, ylabel, yunit, params):
    """Plot a list of params."""
    (p, dock) = self._NewPlot(title)
    p.addLegend()
    p.setLabel('bottom', 'Time')
    p.setLabel('left', ylabel, yunit)
    (pen, pens) = self._GetPlotPen(actuators, [True] * len(params))
    for name, node in actuators:
      for i, param in enumerate(params):
        plot_label = name + ' ' + param
        self._AddLine(node, param,
                      p.plot(name=plot_label, pen=(pen[(node, i)], pens)))
    return dock

  def Redraw(self, data, history):
    """Redraw all plots with new data."""
    data.lock()
    a, b = data.GetIndices(history)
    for node, line in self._lines.items():
      for (name, plot) in line:
        plot.setData(x=data.time[a:b], y=data.GetData(name, node, a, b))
    for node, line in self._bits.items():
      for (name, bit, offset, plot) in line:
        plot.setData(x=data.time[a:b],
                     y=offset + 0.9 * data.GetDataBit(name, bit, node, a, b))
    data.unlock()
    for p in self._plots:
      p.setXRange(-history, 0.0)

  def ClearPlots(self):
    """Clear all plot windows."""
    self._lines = {}
    self._bits = {}
    self._plots = []
    # pylint: disable=invalid-name
    if self.topContainer:
      self.topContainer.close()
    self.topContainer = None

  def CreateCommandPlots(self, servos):
    """Create servo command plots."""
    return [self.CreatePlot(servos, 'Angle command', 'Angle', 'deg',
                            ['ang_m', 'ang_cmd'])]

  def CreateEstimatorPlots(self, servos):
    """Create servo estimator plots."""
    return [self.CreatePlot(servos, 'Angle estimate', 'Angle', 'deg',
                            ['ang_m', 'ang_est']),
            self._PlotVelocity(servos)]

  def CreateStatusPlots(self, servos):
    """Create servo status bit plots."""
    servo_status = [('flags.status', servo_status_bits, 'Paired'),
                    ('flags.status', servo_status_bits, 'Commanded'),
                    ('flags.status', servo_status_bits, 'Armed'),
                    ('flags.status', servo_status_bits, 'Reset'),
                    ('flags.warning', servo_warning_bits, 'PairTimeout'),
                    ('flags.warning', servo_warning_bits, 'PairFailed')]
    r22_supply = ['ShortCircuitDetected',
                  'OverVoltage',
                  'UnderVoltage',
                  'CurrentOutputLimited',
                  'VoltageOutputLimited']
    r22_feedback = ['FeedbackError',
                    'MotorPhasingError',
                    'EnableInputNotActive',
                    'DriveFault']
    return [
        self.CreateLogicPlot(servos, 'Servo status', servo_status),
        self.CreateLogicPlot(
            servos, 'R22 supply',
            [('r22.status_bits', r22_status_bits, x) for x in r22_supply]),
        self.CreateLogicPlot(
            servos, 'R22 feedback',
            [('r22.status_bits', r22_status_bits, x) for x in r22_feedback])]

  def CreateCurrentPlots(self, servos):
    """Create servo current plots."""
    return [self._PlotCurrent(servos)]

  def CreateVoltagePlots(self, servos):
    """Create servo voltage plots."""
    return [self._PlotLineVoltage(servos)]

  def CreatePairedPlots(self, servos):
    """Create paired servo plots."""
    return [self._PlotAngleBias(servos),
            self._PlotAngleError(servos),
            self._PlotVelocityError(servos),
            self._PlotCurrentError(servos)]

  def BuildPlotStack(self, plots, position):
    """Add plots to stack and bring first plot to foreground."""
    self.addDock(plots[0], position)
    for prev, plot in enumerate(plots[1:]):
      self.addDock(plot, 'below', plots[prev])
    # Bring first plot to foreground.
    if len(plots) > 1:
      stack = plots[0].container().stack
      current = stack.currentWidget()
      current.label.setDim(True)
      stack.setCurrentWidget(plots[0])
      plots[0].label.setDim(False)


class MainWindow(QtGui.QMainWindow):
  """Create main window for GUI.

  This class handles the main window, user interface, and plot display.
  """

  def __init__(self, history=60):
    super(MainWindow, self).__init__()
    self._threads = []
    self._history = history
    self._redraw_timer = QtCore.QTimer(self)
    self._InitUserInterface(history)
    self.connect(self._redraw_timer, QtCore.SIGNAL('timeout()'), self._Redraw)

  def _InitUserInterface(self, history):
    """Initialize widgets and layout of user interface."""
    central_widget = QtGui.QWidget(self)

    # Command line.
    command_cbox = QtGui.QComboBox(self)
    command_cbox.setEditable(True)
    command_cbox.lineEdit().returnPressed.connect(self._HandleCommandRequest)
    self._command_cbox = command_cbox

    # Time history.
    history_sbox = QtGui.QSpinBox(self)
    history_sbox.setRange(1, history)
    history_sbox.setSingleStep(1)
    history_sbox.setSuffix(' s')
    history_sbox.setValue(history)
    history_sbox.valueChanged.connect(self._HandleHistoryLength)
    self._history_sbox = history_sbox

    # Refresh rate.
    refresh_sbox = QtGui.QSpinBox(self)
    refresh_sbox.setSuffix(' Hz')
    refresh_sbox.setValue(20)
    refresh_sbox.valueChanged.connect(self._HandleRedrawRate)
    self._refresh_sbox = refresh_sbox

    # Pause button.
    self._pause_btn = QtGui.QPushButton('Pause', self)
    self._pause_btn.clicked.connect(self._HandlePauseButton)

    # Plot area.
    self._plots = PlotDockArea()

    # Status message.
    self._status_message = QtGui.QLabel('', self)

    # Layout.
    hbox = QtGui.QHBoxLayout()
    hbox.addWidget(QtGui.QLabel('Command:', self))
    hbox.addWidget(self._command_cbox, stretch=1)
    hbox.addWidget(QtGui.QLabel('History:', self))
    hbox.addWidget(self._history_sbox)
    hbox.addWidget(QtGui.QLabel('Refresh:', self))
    hbox.addWidget(self._refresh_sbox)
    hbox.addWidget(self._pause_btn)
    vbox = QtGui.QVBoxLayout()
    vbox.addLayout(hbox)
    vbox.addWidget(self._plots, stretch=1)
    central_widget.setLayout(vbox)

    # Main window.
    self.setCentralWidget(central_widget)
    self.setGeometry(300, 150, 1200, 1000)
    self.setWindowTitle('Servo Plotter')
    self.statusBar().addWidget(self._status_message)
    self._SetRedrawRate(refresh_sbox.value())
    self.show()

  def _SelectServoSources(self, sources, history):
    """Close existing plots, then create new plots for specified servos."""
    self._TryCloseThreads()
    self._servo_status = ServoStatusBuffer(allowed_sources=sources,
                                           period=0.01, history=history)
    self._servo_status.start()
    self._data_source = self._servo_status
    self._threads.append(self._servo_status)

  def _SelectMotorSources(self, sources, history):
    """Close existing plots, then create new plots for specified motors."""
    self._TryCloseThreads()
    self._motor_status = MotorStatusBuffer(allowed_sources=sources,
                                           period=0.01, history=history)
    self._motor_status.start()
    self._data_source = self._motor_status
    self._threads.append(self._motor_status)

  def _HandleCommandRequest(self):
    """Handle a user command from text entry."""
    text = self._command_cbox.currentText()
    try:
      command, param = text.split(' ', 1)
    except ValueError:
      command = text
      param = ''
    handlers = {'select': self._HandleSelectCommand}
    command = command.lower()
    if command in handlers:
      handlers[command](param)
      return
    self._PrintError('Unknown command: %s' % command)

  def _HandleSelectCommand(self, param):
    """Handle user select command."""
    params = param.split()

    # Possible nodes to select.
    servo_nodes = [n for n, _ in aio_node_helper
                   if n.startswith('kAioNodeServo')]
    motor_nodes = [n for n, _ in aio_node_helper
                   if n.startswith('kAioNodeMotor')]

    # Parse node selection.
    selected_servos = [s for s in params
                       if 'kAioNodeServo' + s.capitalize() in servo_nodes]
    selected_motors = [m for m in params
                       if 'kAioNodeMotor' + m.capitalize() in motor_nodes]
    for s in selected_servos + selected_motors:
      params.remove(s)

    plots = []
    actuators = []

    if selected_servos and selected_motors:
      # Only one source type can be selected at a time.
      self._PrintError('Only one source type (servos or motors) can be '
                       'selected at a time')
    elif selected_servos:
      # Servos were selected.
      sources = ['kAioNodeServo' + s.capitalize() for s in selected_servos]
      actuators = [(aio_node_helper.ShortName(s), aio_node_helper.Value(s))
                   for s in sources]

      # Select data sources.
      self._SelectServoSources(sources, self._history)

      # Possible plots to select.
      plot_types = {'cmd': (self._plots.CreateCommandPlots, 'top'),
                    'est': (self._plots.CreateEstimatorPlots, 'top'),
                    'status': (self._plots.CreateStatusPlots, 'top'),
                    'cur': (self._plots.CreateCurrentPlots, 'top'),
                    'volt': (self._plots.CreateVoltagePlots, 'top'),
                    'paired': (self._plots.CreatePairedPlots, 'bottom')}

      # Parse plot selection.
      plot_params = [p for p in params if p.lower() in plot_types]
      for p in plot_params:
        params.remove(p)

      # Custom plots.
      for p in [p for p in params if p in self._servo_status.GetParams()]:
        params.remove(p)
        def _GenerateCustomPlot(actuators, p=p):
          return [self._plots.CreatePlot(actuators, p, p, '', [p])]
        plots.append((_GenerateCustomPlot, 'top'))

      # Add default plot selection and add standard plot types to plot list.
      if not plot_params and not plots:
        plot_params = ['cmd']
      for p in plot_params:
        plots.append(plot_types[p.lower()])

      self._PrintMessage('Selected servos: %s' % ', '.join(selected_servos))

    elif selected_motors:
      # Motors were selected.
      sources = ['kAioNodeMotor' + s.capitalize() for s in selected_motors]
      actuators = [(aio_node_helper.ShortName(s), aio_node_helper.Value(s))
                   for s in sources]

      # Select data sources.
      self._SelectMotorSources(sources, self._history)

      # Custom plots.
      for p in [p for p in params if p in self._motor_status.GetParams()]:
        params.remove(p)
        def _GenerateCustomPlot(actuators, p=p):
          return [self._plots.CreatePlot(actuators, p, p, '', [p])]
        plots.append((_GenerateCustomPlot, 'top'))

      self._PrintMessage('Selected motors: %s' % ', '.join(selected_motors))

    else:
      # No nodes were selected.
      self._PrintError('No nodes were selected')

    if params:
      self._PrintError('Unknown parameters: %s' % ' '.join(params))

    # Create plots.
    stacks = {}
    self._plots.ClearPlots()
    for plot in plots:
      func, stack = plot
      if stack not in stacks:
        stacks[stack] = []
      stacks[stack].extend(func(actuators))
    for key, value in stacks.iteritems():
      self._plots.BuildPlotStack(value, key)

  def _HandleRedrawRate(self, value):
    """Handle change to plot refresh rate."""
    self._SetRedrawRate(value)

  def _HandleHistoryLength(self, value):
    """Handle change to history length."""
    pass

  def _HandlePauseButton(self):
    """Handle toggling of pause button."""
    if self._pause_btn.text() == 'Pause':
      self._StopRedraw()
    else:
      self._StartRedraw()

  def _SetRedrawRate(self, hz):
    """Set plot redraw rate."""
    if hz > 0:
      self._redraw_timer.start(int(1000.0 / hz))
      self._StartRedraw()
    else:
      self._StopRedraw()

  def _StartRedraw(self):
    """Start plot redraw timer."""
    self._redraw_timer.start()
    palette = self._pause_btn.palette()
    palette.setColor(QtGui.QPalette.Button, QtCore.Qt.green)
    self._pause_btn.setText('Pause')
    self._pause_btn.setAutoFillBackground(True)
    self._pause_btn.setPalette(palette)
    self._pause_btn.update()

  def _StopRedraw(self):
    """Stop plot redraw timer."""
    self._redraw_timer.stop()
    palette = self._pause_btn.palette()
    palette.setColor(QtGui.QPalette.Button, QtCore.Qt.red)
    self._pause_btn.setText('Paused')
    self._pause_btn.setAutoFillBackground(True)
    self._pause_btn.setPalette(palette)
    self._pause_btn.update()

  def _Redraw(self):
    """Redraw plots."""
    if hasattr(self, '_data_source'):
      self._plots.Redraw(self._data_source, self._history_sbox.value())

  def _PrintMessage(self, msg):
    """Print status message."""
    self._status_message.setText(msg)

  def _PrintError(self, error):
    """Print error message."""
    self._PrintMessage('ERROR: ' + error)

  def _TryCloseThreads(self):
    """Try to close running threads."""
    for thread in self._threads:
      thread.should_exit = True
    for thread in self._threads:
      if thread.isRunning():
        thread.wait(2000)
        if thread.isRunning():
          self._PrintError('Could not terminate {:s}'.format(thread))
          self.close()
    self._threads = []

  def closeEvent(self, event):
    """Override close event in order to close threads."""
    self._TryCloseThreads()
    event.accept()


class AioDataStream(QtCore.QThread, QtCore.QMutex):
  """Handle incoming AIO data.

  This class provides a general interface to handling a circular buffer of
  network data.
  """

  def __init__(self, allowed_sources, message_type, message_template, period,
               history, parent=None):
    QtCore.QThread.__init__(self, parent)
    QtCore.QMutex.__init__(self)

    self.should_exit = False
    self._half_size = int(numpy.ceil(history / period))
    self._buffer_size = 2 * self._half_size
    self._period = period
    self._head = 0
    self._timestamp = time.time()
    self._source_map = {aio_node_helper.Value(x): i
                        for i, x in enumerate(allowed_sources)}
    self._aio_client = aio.AioClient(message_types=[message_type],
                                     allowed_sources=allowed_sources,
                                     timeout=0.2)

    self.time = period * numpy.arange(-self._half_size + 1, 1)
    self._data = [None] * len(allowed_sources)
    self._derived_params = collections.OrderedDict()
    message_dict = ctype_util.CTypeToPython(message_template)

    for i in range(len(self._data)):
      self._data[i] = self._InitBuffers(message_dict)

  def run(self):  # QThread Virtual function.
    """Poll for new messages."""
    while not self.should_exit:
      try:
        (_, header, message) = self._aio_client.Recv()
        self.lock()
        message_dict = ctype_util.CTypeToPython(message)
        self.HandleMessage(header, message_dict, time.time())
        self.unlock()
      except socket.timeout:
        pass
    self._aio_client.Close()

  def HandleMessage(self, header, message, timestamp):
    """Handle new messages."""
    row = self._source_map[header.source]
    dt = timestamp - self._timestamp
    count = min(int(dt / self._period), self._half_size)
    if count:
      # Advance position in circular buffer.
      self.ZeroOrderHold(self._head, count)
      shadow = numpy.mod(self._head + self._half_size, self._buffer_size)
      self.ZeroOrderHold(shadow, count)
      self._head = numpy.mod(self._head + count, self._buffer_size)
      self._timestamp = timestamp
    self.ExtractData(row, self._head, header.type, message)
    shadow = numpy.mod(self._head + self._half_size, self._buffer_size)
    self.ExtractData(row, shadow, header.type, message)

  def _GetNumpyType(self, value):
    if isinstance(value, (float, ctypes.c_float, ctypes.c_double,
                          ctypes.c_longdouble)):
      return numpy.float64
    if isinstance(value, (int, ctypes.c_int, ctypes.c_char, ctypes.c_wchar,
                          ctypes.c_byte, ctypes.c_int)):
      return numpy.int32
    if isinstance(value, (ctypes.c_ubyte, ctypes.c_ushort, ctypes.c_uint,
                          ctypes.c_bool)):
      return numpy.uint32
    if isinstance(value, (long, ctypes.c_long, ctypes.c_longlong)):
      return numpy.int64
    if isinstance(value, (ctypes.c_ulong, ctypes.c_ulonglong)):
      return numpy.uint64
    raise TypeError('Unsupported type encountered in conversion to numpy array')

  def _InitBuffers(self, message, databuf=None, prefix=''):
    if databuf is None:
      databuf = {}
    if isinstance(message, dict):
      for param, value in message.iteritems():
        self._InitBuffers(value, databuf, prefix + '.' + param)
    elif isinstance(message, list):
      for i in range(len(message)):
        self._InitBuffers(message[i], databuf, prefix + '[%d]' % i)
    elif message is not None and not isinstance(message, str):
      databuf[prefix[1:]] = numpy.zeros(self._buffer_size,
                                        dtype=self._GetNumpyType(message))
    return databuf

  def _ExtractData(self, col, message, databuf, prefix=''):
    if isinstance(message, dict):
      for param, value in message.iteritems():
        self._ExtractData(col, value, databuf,
                          prefix + '.' + param)
    elif isinstance(message, list):
      for i in range(len(message)):
        self._ExtractData(col, message[i], databuf, prefix + '[%d]' % i)
    elif message is not None and not isinstance(message, str):
      databuf[prefix[1:]][col] = message

  def ExtractData(self, source, col, message_type, message):
    """Extract data from message and insert into circular buffer."""
    self._ExtractData(col, message, self._data[source])

    # Derive parameters.
    for param_name, operation in self._derived_params.iteritems():
      for source in range(len(self._source_map)):
        self._data[source][param_name][col] = operation(col, source, self._data)

  def ZeroOrderHold(self, start, count):
    """Duplicate data at start index for the following count indices."""

    for databuf in self._data:
      for arr in databuf.values():
        for index in range(start + 1, start + 1 + count):
          arr[index % self._buffer_size] = arr[start]

  def DeriveParam(self, name, operation):
    """Derive a new parameter based on other message parameters.

    Derived parameters are referred to by name, and consist of an operation to
    be performed on input data.  The operation has a function which performs
    the actual computation to produce a specific sample of that parameter, and
    the data type which is used to allocate the array containing the resulting
    data values.

    Args:
        name: The name used to plot the derived parameter.
        operation: The calculation applied to the data buffer to compute the
            derived parameter value.
    """
    if isinstance(operation, Alias):
      for i in range(len(self._data)):
        self._data[i][name] = self._data[i][operation.base_name]
    elif isinstance(operation, Operation):
      if name in self._derived_params:
        del self._derived_params[name]

      self._derived_params[name] = operation.function
      for i in range(len(self._data)):
        self._data[i][name] = numpy.zeros(self._buffer_size,
                                          dtype=operation.dtype)

  def GetIndices(self, history):
    """Get circular buffer indices for a given buffer length (history)."""
    b = self._half_size
    a = max(int(numpy.ceil(self._half_size - history / self._period)), 0)
    return (a, b)

  def GetData(self, name, node, a, b):
    """Extract data from circular buffer."""
    offset = numpy.mod(self._head, self._half_size) + 1
    row = self._source_map[node]
    a += offset
    b += offset
    return self._data[row][name][a:b]

  def GetDataBit(self, name, bit, node, a, b):
    """Extract data from a bitmask from circular buffer."""
    offset = numpy.mod(self._head, self._half_size) + 1
    row = self._source_map[node]
    a += offset
    b += offset
    return (self._data[row][name][a:b] & (1 << bit)) >> bit

  def GetParams(self):
    return self._data[0].keys()


class MotorStatusBuffer(AioDataStream):
  """Handle incoming AIO data with message type of MotorStatusMessage."""

  def __init__(self, allowed_sources, period, history, parent=None):
    """Initialize circular buffer data storage."""
    super(MotorStatusBuffer, self).__init__(
        message_type='kMessageTypeMotorStatus',
        message_template=avionics_messages.MotorStatusMessage(),
        allowed_sources=allowed_sources,
        period=period, history=history, parent=parent)

    self.DeriveParam('vref', Operation(
        lambda c, s, d: d[s]['vd'][c]**2 + d[s]['vq'][c]**2,
        numpy.float64))


class ServoStatusBuffer(AioDataStream):
  """Handle incoming AIO data with message type of ServoStatusMessage."""

  def __init__(self, allowed_sources, period, history, parent=None):
    """Initialize circular buffer data storage."""
    super(ServoStatusBuffer, self).__init__(
        message_type='kMessageTypeServoStatus',
        message_template=avionics_messages.ServoStatusMessage(),
        allowed_sources=allowed_sources,
        period=period, history=history, parent=parent)

    self.DeriveParam('ang_m', RadiansToDegreesOp('angle_measured'))
    self.DeriveParam('ang_err', DeltaFirstSourceOp('ang_m'))
    self.DeriveParam('ang_bias', RadiansToDegreesOp('angle_bias'))
    self.DeriveParam('ang_fb', RadiansToDegreesOp('angle_feedback'))
    self.DeriveParam('ang_est', RadiansToDegreesOp('angle_estimate'))
    self.DeriveParam('ang_var', RadiansToDegreesOp('angle_variance'))
    self.DeriveParam('ang_cmd', RadiansToDegreesOp('angle_desired'))
    self.DeriveParam('vel_m', RadiansToDegreesOp('angular_velocity'))
    self.DeriveParam('vel_err', DeltaFirstSourceOp('vel_m'))
    self.DeriveParam('cur_m', MultiplyOp('r22.current', 0.01))
    self.DeriveParam('cur_err', DeltaFirstSourceOp('cur_m'))
    self.DeriveParam('cur_limit', MultiplyOp('r22.current_limit', 0.01))
    self.DeriveParam('cur_nlimit', MultiplyOp('cur_limit', -1))
    self.DeriveParam('i_servo', Alias(
        'servo_mon.analog_data[%d]'
        % servo_monitor_types.kServoAnalogVoltageIServo))
    self.DeriveParam('v_lv_in_a', Alias(
        'servo_mon.analog_data[%d]'
        % servo_monitor_types.kServoAnalogVoltageLvA))
    self.DeriveParam('v_lv_in_b', Alias(
        'servo_mon.analog_data[%d]'
        % servo_monitor_types.kServoAnalogVoltageLvB))
    self.DeriveParam('v_servo', Alias(
        'servo_mon.analog_data[%d]'
        % servo_monitor_types.kServoAnalogVoltageVServo))


def _HandleSigint(*unused_args):  # pylint: disable=invalid-name
  """Quit the application when SIGINT is received."""
  sys.stderr.write('\n')
  QtGui.QApplication.quit()


def main(argv):
  """Entry point."""
  signal.signal(signal.SIGINT, _HandleSigint)
  app = QtGui.QApplication(argv)
  unused_win = MainWindow()
  sys.exit(app.exec_())


if __name__ == '__main__':
  main(sys.argv)
