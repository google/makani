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


"""A script for plotting various data sets from the motor controllers."""

import collections
import importlib
import math
import Queue
import socket
import sys
import time
import warnings

from makani.avionics.common import aio
from makani.avionics.common import motor_adc_defines as adc_defines
from makani.avionics.common import motor_thermal_types
import numpy
from PySide import QtCore
from PySide import QtGui

# Must be included after PySide in order to force pyqtgraph to use it.
pyqtgraph = importlib.import_module('pyqtgraph')
dockarea = importlib.import_module('pyqtgraph.dockarea')


class PlotterMainWindow(QtGui.QMainWindow):
  """The main GUI window for motor data plotting.

  This class handles the setup and plotting of various types of graphs for
  motor debug and analysis.
  """

  _REALTIME_DATA_LEN = 3000

  def __init__(self):
    """Creates a new GUI window instance."""
    super(PlotterMainWindow, self).__init__()
    self._threads = []
    self._InitUI()
    self._is_paused = False

  def _InitUI(self):
    """Create and arrange all GUI elements."""
    central_widget = QtGui.QWidget(self)

    log_path_cbox = QtGui.QComboBox(self)
    log_path_cbox.setEditable(True)
    log_path_cbox.lineEdit().returnPressed.connect(self._HandleLogRequest)

    downsample_ledit = QtGui.QLineEdit(self)
    downsample_ledit.setValidator(QtGui.QIntValidator(1, 10000))
    downsample_ledit.returnPressed.connect(
        log_path_cbox.lineEdit().returnPressed)

    load_btn = QtGui.QPushButton('Load', self)
    load_btn.clicked.connect(log_path_cbox.lineEdit().returnPressed)

    pause_btn = QtGui.QPushButton('Pause', self)
    pause_btn.clicked.connect(self._HandlePauseRequest)

    plot_dockarea = dockarea.DockArea()

    status_message = QtGui.QLabel('', self)

    hbox = QtGui.QHBoxLayout()
    hbox.addWidget(QtGui.QLabel('Log File:', self))
    hbox.addWidget(log_path_cbox, stretch=1)
    hbox.addWidget(QtGui.QLabel('Downsample:', self))
    hbox.addWidget(downsample_ledit)
    hbox.addWidget(load_btn)
    hbox.addWidget(pause_btn)

    vbox = QtGui.QVBoxLayout()
    vbox.addLayout(hbox)
    vbox.addWidget(plot_dockarea, stretch=1)

    central_widget.setLayout(vbox)

    self.setCentralWidget(central_widget)
    self.setGeometry(300, 150, 1200, 1000)
    self.setWindowTitle('Motor Plotter')
    self.statusBar().addWidget(status_message)
    self.show()

    self._downsample_ledit = downsample_ledit
    self._status_message = status_message
    self._log_path_cbox = log_path_cbox
    self._plot_dockarea = plot_dockarea

  def _IsValidMotor(self, m):
    return m.lower() in ['sbo', 'sbi', 'pbi', 'pbo', 'pto', 'pti', 'sti', 'sto']

  def _HandlePauseRequest(self):
    """Handle user request to pause data."""
    self._is_paused = not self._is_paused

  def _HandleLogRequest(self):
    """Handle user request to plot data.

    This method is attached to the return press and 'Load' button press events.
    It handles various requests differently based on the string in the log
    path combo box.
    """
    self._PrintMessage('Handling Loading Log.')

    text = self._log_path_cbox.currentText()
    downsample = self._downsample_ledit.text()

    try:
      command, motor = text.split(None, 1)
    except ValueError:
      self._PrintError('Not enough arguments.')
      return

    if not self._IsValidMotor(motor):
      self._PrintError('"{:s}" is not a valid motor.'.format(motor))
      return

    if command.lower() not in ['realtime', 'control', 'adc']:
      self._PrintError('Unknown command: "{:s}"'.format(command))
      return

    # Close old threads and create new data queue.
    self._TryCloseThreads()
    self._listener_data_queue = Queue.Queue(maxsize=1000)

    # Real-time request.
    if command.lower() == 'realtime':
      if not downsample or int(downsample) == 0:
        self._PrintError('No downsample givien.')
        return

      downsample = int(downsample)
      buffer_size = math.ceil(5.0/downsample)

      self._PrintMessage(('Starting realtime plotter. Downsample: '
                          '{:d} Motor: {:s}').format(downsample, motor))

      self._SetupRealtimeData(downsample)
      self._SetupStatusPlot()

      for plot in self._plot_items:
        plot.setXRange(-0.01 * self._REALTIME_DATA_LEN * downsample, 0)

      status_listener = StatusListener(downsample, buffer_size,
                                       'kAioNodeMotor' + motor.capitalize(),
                                       self._listener_data_queue)
      self._threads.append(status_listener)

      status_listener.has_data.connect(self._PlotRealtimeData)
      status_listener.has_error.connect(self._PrintError)
      status_listener.start()

    # High-fidelity control logs.
    elif command.lower() == 'control':
      self._PrintMessage('Starting control plotter. Motor: {:s}'.format(motor))

      self._SetupControlPlot()

      control_listener = ControlDataListener(
          'kAioNodeMotor' + motor.capitalize(),
          self._listener_data_queue)
      self._threads.append(control_listener)

      control_listener.has_data.connect(self._PlotControlData)
      control_listener.has_error.connect(self._PrintError)
      control_listener.start()

    # Raw ADC counts
    elif command.lower() == 'adc':
      self._PrintMessage('Starting ADC plotter. Motor: ' + motor)

      self._SetupAdcPlot()

      adc_listener = AdcDataListener(
          'kAioNodeMotor' + motor.capitalize(),
          self._listener_data_queue)
      self._threads.append(adc_listener)

      adc_listener.has_data.connect(self._PlotAdcData)
      adc_listener.has_error.connect(self._PrintError)
      adc_listener.start()

  def _PlotAdcData(self):
    """Plot ADC data from listener.

    This method is called by a signal from a listener thread that indicates
    there is new data in the queue to be plotted.
    """
    data = self._listener_data_queue.get()
    for key, plot in self._plots.iteritems():
      # This filter catches the warning: "Item size computed from the PEP 3118
      # buffer format string does not match the actual item size."
      # This is a problem with numpy.ctypeslib.as_array().
      with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        plot.setData(x=numpy.arange(adc_defines.NUM_ADC_SAMPLES),
                     y=numpy.ctypeslib.as_array(getattr(data, key)))

  def _PlotControlData(self):
    """Plot control data from listener.

    This method is called by a signal from a listener thread that indicates
    there is new data in the queue to be plotted.
    """
    packet_list = self._listener_data_queue.get()
    length = len(packet_list)
    data = {key: numpy.zeros(length)
            for key in self._plots.keys() + ['timestep']}

    for ind, pkt in enumerate(packet_list):
      for key in data.keys():
        data[key][ind] = getattr(pkt, key)

    for key, plot in self._plots.iteritems():
      plot.setData(x=data['timestep'], y=data[key])

  def _PlotRealtimeData(self):
    """Plot real-time status data from listener.

    This method is called by a signal from a listener thread that indicates
    thare is new real-time data in the queue to be plotted.  The data is plotted
    in a rolling circular buffer.
    """
    buffer_count = 0
    while True:
      try:
        msg = self._listener_data_queue.get_nowait()
      except Queue.Empty:
        break
      self._realtime_data['bus_current'][buffer_count] = msg.bus_current
      self._realtime_data['bus_voltage'][buffer_count] = msg.bus_voltage
      self._realtime_data['vref'][buffer_count] = (
          math.sqrt(msg.vd**2 + msg.vq**2))
      self._realtime_data['omega'][buffer_count] = msg.omega
      self._realtime_data['omega_upper_limit'][buffer_count] = (
          msg.omega_upper_limit)
      self._realtime_data['omega_lower_limit'][buffer_count] = (
          msg.omega_lower_limit)
      self._realtime_data['torque_cmd'][buffer_count] = msg.torque_cmd
      self._realtime_data['iq'][buffer_count] = msg.iq
      self._realtime_data['iq_cmd'][buffer_count] = msg.iq_cmd
      self._realtime_data['id'][buffer_count] = msg.id
      self._realtime_data['id_cmd'][buffer_count] = msg.id_cmd
      self._realtime_data['vd'][buffer_count] = msg.vd
      self._realtime_data['vq'][buffer_count] = msg.vq
      self._realtime_data['current_correction'][buffer_count] = (
          msg.current_correction)
      self._realtime_data['speed_correction'][buffer_count] = (
          msg.speed_correction)
      self._realtime_data['voltage_pair_bias'][buffer_count] = (
          msg.voltage_pair_bias)
      self._realtime_data['board_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelBoard])
      self._realtime_data['controller_air_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelControllerAir])
      self._realtime_data['stator_core_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelStatorCore])
      self._realtime_data['stator_coil_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelStatorCoil])
      self._realtime_data['nacelle_air_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelNacelleAir])
      self._realtime_data['rotor_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelRotor])
      self._realtime_data['heat_plate_1_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelHeatPlate1])
      self._realtime_data['heat_plate_2_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelHeatPlate2])
      self._realtime_data['capacitor_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelCapacitor])
      self._realtime_data['ht3000a_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelHt3000A])
      self._realtime_data['ht3000b_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelHt3000B])
      self._realtime_data['ht3000c_temp'][buffer_count] = (
          msg.temps[motor_thermal_types.kMotorThermalChannelHt3000C])

      buffer_count += 1

    for key, plot in self._plots.items():
      self._realtime_data[key] = numpy.roll(self._realtime_data[key],
                                            -buffer_count)
      if not self._is_paused:
        plot.setData(x=self._realtime_x, y=self._realtime_data[key])

  def _SetupRealtimeData(self, downsample):
    """Setup data structure for real-time data plotting.

    Args:
      downsample: An integer specifying the downsample ratio for the incoming
        real-time data.
    """
    self._realtime_x = 0.01 * downsample * numpy.arange(
        -self._REALTIME_DATA_LEN + 1, 1)
    self._realtime_data = collections.defaultdict(
        lambda: numpy.zeros(self._REALTIME_DATA_LEN))

  def _CleanDockArea(self):
    """Removes all containers and docks from self._plot_dockarea."""
    (_, docks) = self._plot_dockarea.findAll()
    for d in docks.values():
      d.hide()
      d.setParent(None)
      d.label.setParent(None)

  def _SetupAdcPlot(self):
    """Setup all necessary plot GUI items for plotting ADC data."""
    self._plots = {}
    self._plot_items = []

    self._CleanDockArea()

    dock1 = dockarea.Dock('ADC Values')
    self._plot_dockarea.addDock(dock1, 'top')
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock1.addWidget(glw)

    plot1 = glw.addPlot(title='Raw ADC Counts')
    plot1.addLegend()
    self._plots['v_in_monitor'] = plot1.plot(name='vin', pen='r')
    self._plots['phase_a_current'] = plot1.plot(name='ia', pen='g')
    self._plots['phase_b_current'] = plot1.plot(name='ib', pen='b')
    self._plots['phase_b_aux_current'] = plot1.plot(name='ib_aux', pen='c')
    self._plots['phase_c_current'] = plot1.plot(name='ic', pen='m')
    self._plots['bus_current'] = plot1.plot(name='ibus', pen='y')
    self._plots['bus_voltage'] = plot1.plot(name='vbus', pen='w')
    self._plots['v_aux_monitor'] = plot1.plot(name='vaux', pen=(255, 150, 0))
    plot1.setLabel('bottom', 'Sample')
    plot1.setLabel('left', 'Count')
    plot1.showGrid(True, True)
    self._plot_items.append(plot1)

  def _SetupControlPlot(self):
    """Setup all necessary plot GUI items for plotting control data."""
    self._plots = {}
    self._plot_items = []

    self._CleanDockArea()

    dock1 = dockarea.Dock('Motor Currents')
    self._plot_dockarea.addDock(dock1, 'top')
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock1.addWidget(glw)

    plot1 = glw.addPlot(title='Phase Currents')
    plot1.addLegend()
    self._plots['motor_state_ia'] = plot1.plot(name='ia', pen='r')
    self._plots['motor_state_ib'] = plot1.plot(name='ib', pen='g')
    self._plots['motor_state_ic'] = plot1.plot(name='ic', pen='b')
    plot1.setLabel('bottom', 'Timestep')
    plot1.setLabel('left', 'Current', 'A')
    plot1.showGrid(True, True)
    self._plot_items.append(plot1)

    glw.nextRow()

    plot2 = glw.addPlot(title='FOC Currents')
    plot2.addLegend()
    self._plots['foc_current_actual_iq'] = plot2.plot(name='iq_actual',
                                                      pen='r')
    self._plots['foc_current_actual_id'] = plot2.plot(name='id_actual',
                                                      pen='g')
    self._plots['foc_current_desired_iq'] = plot2.plot(name='iq_desired',
                                                       pen='b')
    self._plots['foc_current_desired_id'] = plot2.plot(name='id_desired',
                                                       pen='c')
    plot2.setLabel('bottom', 'Timestep')
    plot2.setLabel('left', 'Current', 'A')
    plot2.showGrid(True, True)
    plot2.setXLink(plot1)
    self._plot_items.append(plot2)

    dock2 = dockarea.Dock('SVPWM Output')
    self._plot_dockarea.addDock(dock2, 'below', dock1)
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock2.addWidget(glw)

    plot3 = glw.addPlot(title='SVPWM Voltage')
    plot3.addLegend()
    self._plots['foc_voltage_vd'] = plot3.plot(name='vd', pen='r')
    self._plots['foc_voltage_vq'] = plot3.plot(name='vq', pen='g')
    self._plots['foc_voltage_v_ref'] = plot3.plot(name='vref', pen='b')
    plot3.setLabel('bottom', 'Timestep')
    plot3.setLabel('left', 'Voltage', 'V')
    plot3.setXLink(plot1)
    plot3.showGrid(True, True)
    self._plot_items.append(plot3)

    glw.nextRow()

    plot4 = glw.addPlot(title='Voltage Angle')
    self._plots['foc_voltage_angle'] = plot4.plot(name='vangle', pen='r')
    plot4.setLabel('bottom', 'Timestep')
    plot4.setLabel('left', 'Angle', 'rad')
    plot4.showGrid(True, True)
    plot4.setXLink(plot1)
    self._plot_items.append(plot4)

    dock3 = dockarea.Dock('Motor Position')
    self._plot_dockarea.addDock(dock3, 'below', dock1)
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock3.addWidget(glw)

    plot5 = glw.addPlot(title='Motor Theta')
    self._plots['motor_state_theta_elec'] = plot5.plot(name='theta', pen='r')
    plot5.setLabel('bottom', 'Timestep')
    plot5.setLabel('left', 'Theta', 'rad')
    plot5.showGrid(True, True)
    plot5.setXLink(plot1)
    self._plot_items.append(plot5)

    glw.nextRow()

    plot6 = glw.addPlot(title='Motor Omega')
    plot6.addLegend()
    self._plots['motor_state_omega_mech'] = plot6.plot(name='omega_actual',
                                                       pen='r')
    self._plots['omega_upper_limit'] = plot6.plot(name='omega_upper_limit',
                                                  pen='g')
    self._plots['omega_lower_limit'] = plot6.plot(name='omega_lower_limit',
                                                  pen='b')
    plot6.setLabel('bottom', 'Timestep')
    plot6.setLabel('left', 'Omega', 'rad/s')
    plot6.showGrid(True, True)
    plot6.setXLink(plot1)
    self._plot_items.append(plot6)

    dock4 = dockarea.Dock('I Controller State')
    self._plot_dockarea.addDock(dock4, 'below', dock1)
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock4.addWidget(glw)

    plot7 = glw.addPlot(title='Integral State')
    plot7.addLegend()
    self._plots['foc_state_id_int'] = plot7.plot(name='i_int_d', pen='r')
    self._plots['foc_state_iq_int'] = plot7.plot(name='i_int_q', pen='g')
    plot7.setLabel('bottom', 'Timestep')
    plot7.setLabel('left', 'State')
    plot7.showGrid(True, True)
    plot7.setXLink(plot1)
    self._plot_items.append(plot7)

    glw.nextRow()

    plot8 = glw.addPlot(title='Phase Current Error')
    plot8.addLegend()
    self._plots['foc_state_id_error'] = plot8.plot(name='id_error', pen='r')
    self._plots['foc_state_iq_error'] = plot8.plot(name='iq_error', pen='g')
    plot8.setLabel('bottom', 'Timestep')
    plot8.setLabel('left', 'Error')
    plot8.showGrid(True, True)
    plot8.setXLink(plot1)
    self._plot_items.append(plot8)

    dock5 = dockarea.Dock('W Controller State')
    self._plot_dockarea.addDock(dock5, 'below', dock1)
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock5.addWidget(glw)

    plot9 = glw.addPlot(title='Integral State')
    self._plots['foc_state_omega_int'] = plot9.plot(name='w_int', pen='r')
    plot9.setLabel('bottom', 'Timestep')
    plot9.setLabel('left', 'State')
    plot9.showGrid(True, True)
    plot9.setXLink(plot1)
    self._plot_items.append(plot9)

    glw.nextRow()

    plot10 = glw.addPlot(title='Last Error')
    self._plots['foc_state_omega_error_last'] = plot10.plot(name='w_last_error',
                                                            pen='r')
    plot10.setLabel('bottom', 'Timestep')
    plot10.setLabel('left', 'Error')
    plot10.showGrid(True, True)
    plot10.setXLink(plot1)
    self._plot_items.append(plot10)

    dock6 = dockarea.Dock('Input Bus')
    self._plot_dockarea.addDock(dock6, 'below', dock1)
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock6.addWidget(glw)

    plot11 = glw.addPlot(title='Input Voltage')
    self._plots['motor_state_v_bus'] = plot11.plot(name='vbus', pen='r')
    plot11.setLabel('bottom', 'Timestep')
    plot11.setLabel('left', 'Voltage', 'V')
    plot11.showGrid(True, True)
    plot11.setXLink(plot1)
    self._plot_items.append(plot11)

    glw.nextRow()

    plot12 = glw.addPlot(title='Input Current')
    self._plots['motor_state_i_bus'] = plot12.plot(name='ibus', pen='r')
    plot12.setLabel('bottom', 'Timestep')
    plot12.setLabel('left', 'Current', 'A')
    plot12.showGrid(True, True)
    plot12.setXLink(plot1)
    self._plot_items.append(plot12)

    # Raise Motor Current dock to foreground
    stack = dock1.container().stack
    current = stack.currentWidget()
    current.label.setDim(True)
    stack.setCurrentWidget(dock1)
    dock1.label.setDim(False)

  def _SetupStatusPlot(self):
    """Setup all necessary plot GUI items for plotting status data."""
    self._plots = {}
    self._plot_items = []

    self._CleanDockArea()

    dock1 = dockarea.Dock('Omega & Current')
    self._plot_dockarea.addDock(dock1, 'top')
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock1.addWidget(glw)

    plot1 = glw.addPlot(title='Mechanical Omega')
    plot1.addLegend()
    self._plots['omega'] = plot1.plot(name='Omega', pen='g')
    self._plots['omega_upper_limit'] = plot1.plot(name='Omega Upper Limit',
                                                  pen=(255, 0, 0))
    self._plots['omega_lower_limit'] = plot1.plot(name='Omega Lower Limit',
                                                  pen=(150, 0, 0))
    plot1.setLabel('bottom', 'Time', 's')
    plot1.setLabel('left', 'Omega', 'rad/s')
    plot1.showGrid(True, True)
    self._plot_items.append(plot1)

    glw.nextRow()

    plot2 = glw.addPlot(title='Motor Current')
    plot2.addLegend()
    self._plots['iq'] = plot2.plot(name='iq', pen='r')
    self._plots['iq_cmd'] = plot2.plot(name='iq cmd', pen=(150, 0, 0))
    self._plots['id'] = plot2.plot(name='id', pen='g')
    self._plots['id_cmd'] = plot2.plot(name='id cmd', pen=(0, 100, 0))
    plot2.setLabel('bottom', 'Time', 's')
    plot2.setLabel('left', 'Current', 'A')
    plot2.showGrid(True, True)
    plot2.setXLink(plot1)
    self._plot_items.append(plot2)

    dock2 = dockarea.Dock('Voltage & Temp')
    self._plot_dockarea.addDock(dock2, 'below', dock1)
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock2.addWidget(glw)

    plot3 = glw.addPlot(title='SVPWM Voltage')
    plot3.addLegend()
    self._plots['vd'] = plot3.plot(name='vd', pen='r')
    self._plots['vq'] = plot3.plot(name='vq', pen='g')
    self._plots['vref'] = plot3.plot(name='vref', pen='b')
    plot3.setLabel('bottom', 'Time', 's')
    plot3.setLabel('left', 'Voltage', 'V')
    plot3.setXLink(plot1)
    plot3.showGrid(True, True)
    self._plot_items.append(plot3)

    glw.nextRow()

    plot4 = glw.addPlot(title='Temperatures')
    plot4.addLegend()
    self._plots['board_temp'] = plot4.plot(name='Board', pen='r')
    self._plots['controller_air_temp'] = plot4.plot(name='Controller Air',
                                                    pen=(0, 100, 0))
    self._plots['stator_core_temp'] = plot4.plot(name='Motor Stator Core',
                                                 pen='g')
    self._plots['stator_coil_temp'] = plot4.plot(name='Motor Stator Coil',
                                                 pen='b')
    self._plots['nacelle_air_temp'] = plot4.plot(name='Nacelle Air', pen='c')
    self._plots['rotor_temp'] = plot4.plot(name='Motor Rotor', pen='w')
    self._plots['heat_plate_1_temp'] = plot4.plot(name='Heat Plate 1', pen='m')
    self._plots['heat_plate_2_temp'] = plot4.plot(name='Heat Plate 2', pen='y')
    self._plots['capacitor_temp'] = plot4.plot(name='Capacitor',
                                               pen=(255, 128, 0))
    self._plots['ht3000a_temp'] = plot4.plot(name='Ht3000A',
                                             pen=(11, 224, 195))
    self._plots['ht3000b_temp'] = plot4.plot(name='Ht3000B',
                                             pen=(11, 156, 224))
    self._plots['ht3000c_temp'] = plot4.plot(name='Ht3000C',
                                             pen=(11, 78, 224))
    plot4.setLabel('bottom', 'Time', 's')
    plot4.setLabel('left', 'Temperature', 'degC')
    plot4.showGrid(True, True)
    plot4.setXLink(plot1)
    self._plot_items.append(plot4)

    dock3 = dockarea.Dock('Input Bus')
    self._plot_dockarea.addDock(dock3, 'below', dock1)
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock3.addWidget(glw)

    plot5 = glw.addPlot(title='Input Voltage')
    self._plots['bus_voltage'] = plot5.plot(name='Input Voltage', pen='r')
    plot5.setLabel('bottom', 'Time', 's')
    plot5.setLabel('left', 'Voltage', 'V')
    plot5.showGrid(True, True)
    plot5.setXLink(plot1)
    self._plot_items.append(plot5)

    glw.nextRow()

    plot6 = glw.addPlot(title='Input Current')
    self._plots['bus_current'] = plot6.plot(name='Input Current', pen='r')
    plot6.setLabel('bottom', 'Time', 's')
    plot6.setLabel('left', 'Current', 'A')
    plot6.showGrid(True, True)
    plot6.setXLink(plot1)
    self._plot_items.append(plot6)

    dock4 = dockarea.Dock('Stacking')
    self._plot_dockarea.addDock(dock4, 'below', dock3)
    glw = pyqtgraph.GraphicsLayoutWidget()
    dock4.addWidget(glw)

    plot7 = glw.addPlot(title='Speed Correction')
    self._plots['speed_correction'] = plot7.plot(name='Speed Correction',
                                                 pen='g')
    plot7.setLabel('bottom', 'Time', 's')
    plot7.setLabel('left', 'Omega', 'rad/s')
    plot7.showGrid(True, True)
    plot7.setXLink(plot1)
    self._plot_items.append(plot7)

    glw.nextRow()

    plot8 = glw.addPlot(title='Voltage Pair Bias')
    self._plots['voltage_pair_bias'] = plot8.plot(name='Voltage Pair Bias',
                                                  pen='r')
    plot8.setLabel('bottom', 'Time', 's')
    plot8.setLabel('left', 'Voltage', 'V')
    plot8.showGrid(True, True)
    plot8.setXLink(plot1)
    self._plot_items.append(plot8)

    glw.nextRow()

    plot9 = glw.addPlot(title='Current Correction')
    self._plots['current_correction'] = plot9.plot(name='Current Correction',
                                                   pen='r')
    plot9.setLabel('bottom', 'Time', 's')
    plot9.setLabel('left', 'Current', 'A')
    plot9.showGrid(True, True)
    plot9.setXLink(plot1)
    self._plot_items.append(plot9)

    # Raise Omega & Current dock to foreground
    stack = dock1.container().stack
    current = stack.currentWidget()
    current.label.setDim(True)
    stack.setCurrentWidget(dock1)
    dock1.label.setDim(False)

  def _PrintMessage(self, msg):
    """Print message in status bar and stdout."""
    self._status_message.setText(msg)
    print msg

  def _PrintError(self, error):
    """Print error message."""
    self._PrintMessage('ERROR: ' + error)

  def _TryCloseThreads(self):
    """Try to close running threads."""
    for thread in self._threads:
      if thread.isRunning():
        thread.should_exit = True
        thread.wait(2000)
        if thread.isRunning():
          self._PrintError('Could not terminate {:s}'.format(thread))
          self.close()
    self._threads = []

  def closeEvent(self, event):
    """Overide close event in order to close threads.

    Args:
      event: A GUI event.
    """
    self._TryCloseThreads()
    event.accept()


class AdcDataListener(QtCore.QThread):
  """A thread that listens for ADC data.

  Attributes:
    has_error: A signal used to communicate to other threads that the
      AdcDataListener thread has encountered an error.  An error string
      is passed with the signal.
    has_data: A signal used to communicate to other threads that the
      AdcDataListener thread has new data in the queue provided during
      instantiation.
    should_exit: A boolean indicating if the thread should stop executing.
  """
  # These are class members that via some magic create
  # instance members of the same name.  Magic here:
  # http://qt-project.org/wiki/Signals_and_Slots_in_PySide
  has_error = QtCore.Signal(str)
  has_data = QtCore.Signal()

  def __init__(self, source, data_queue, parent=None):
    """Initialize an AdcDataListener.

    Args:
      source: A string specifying which AIO node to listen to.
      data_queue: A Queue.Queue used to pass data back to the instantiating
        thread in a thread-safe manner.
      parent: An optional parent argument for QtCore.QThread.
    """
    QtCore.QThread.__init__(self, parent)

    self.should_exit = False
    self._data_queue = data_queue
    self._aio_client = aio.AioClient(['kMessageTypeMotorAdcLog'],
                                     allowed_sources=[source], timeout=0.1)

  def run(self):
    """Run in a separate thread."""
    while not self.should_exit:
      try:
        (_, _, msg) = self._aio_client.Recv()
        self._data_queue.put(msg)
        self.has_data.emit()
      except socket.timeout:
        pass
    self._aio_client.Close()


class ControlDataListener(QtCore.QThread):
  """A thread that listens for control data.

  Attributes:
    has_error: A signal used to communicate to other threads that the
      ControlDataListener thread has encountered an error.  An error string
      is passed with the signal.
    has_data: A signal used to communicate to other threads that the
      ControlDataListener thread has new data in the queue provided during
      instantiation.
    should_exit: A boolean indicating if the thread should stop executing.
  """
  # These are class members that via some magic create
  # instance members of the same name.  Magic here:
  # http://qt-project.org/wiki/Signals_and_Slots_in_PySide
  has_error = QtCore.Signal(str)
  has_data = QtCore.Signal()

  def __init__(self, source, data_queue, parent=None):
    """Initialize a ControlDataListener.

    Args:
      source: A string specifying which AIO node to listen to.
      data_queue: A Queue.Queue used to pass data back to the instantiating
        thread in a thread-safe manner.
      parent: An optional parent argument for QtCore.QThread.
    """
    QtCore.QThread.__init__(self, parent)

    self.should_exit = False
    self._data_queue = data_queue
    self._aio_client = aio.AioClient(['kMessageTypeMotorIsrLog'],
                                     allowed_sources=[source], timeout=0.1)
    self._timeout = None
    self._data_list = []

  def run(self):
    """Run in a separate thread."""
    while not self.should_exit:
      try:
        # Time to send data?
        if self._timeout and self._timeout < time.time():
          self._timeout = None
          self._data_list.sort(key=(lambda x: x.timestep))
          self._data_queue.put(self._data_list)
          self._data_list = []
          self.has_data.emit()
        # Receive data.
        (_, _, msg) = self._aio_client.Recv()
        self._data_list.append(msg)
        self._timeout = time.time() + 0.2
      except socket.timeout:
        pass
    self._aio_client.Close()


class StatusListener(QtCore.QThread):
  """A thread that listens for status data.

  Attributes:
    has_error: A signal used to communicate to other threads that the
      ControlDataListener thread has encountered an error.  An error string
      is passed with the signal.
    has_data: A signal used to communicate to other threads that the
      ControlDataListener thread has new data in the queue provided during
      instantiation.
    should_exit: A boolean indicating if the thread should stop executing.
  """
  # These are class members that via some magic create
  # instance members of the same name.  Magic here:
  # http://qt-project.org/wiki/Signals_and_Slots_in_PySide
  has_error = QtCore.Signal(str)
  has_data = QtCore.Signal()

  def __init__(self, downsample, buffer_size, source, data_queue, parent=None):
    """Initialize a StatusListener.

    Args:
      downsample: An integer specifying the subsample ratio of the data that is
        passed back via the queue.
      buffer_size: An integer specifying how many data points are added to the
        queue before the thread emits its has_data signal.
      source: A string specifying which AIO node to listen to.
      data_queue: A Queue.Queue used to pass data back to the instantiating
        thread in a thread-safe manner.
      parent: An optional parent argument for QtCore.QThread.
    """
    QtCore.QThread.__init__(self, parent)

    self.should_exit = False
    self._downsample = downsample
    self._buffer_size = buffer_size
    self._data_queue = data_queue
    self._aio_client = aio.AioClient(['kMessageTypeMotorStatus'],
                                     allowed_sources=[source], timeout=0.2)

  def run(self):
    """Run in a separate thread."""
    downsample_counter = 0
    buffer_counter = 0

    while not self.should_exit:
      try:
        if downsample_counter < self._downsample - 1:
          downsample_counter += 1
          self._aio_client.Recv()
        else:
          downsample_counter = 0
          (_, _, msg) = self._aio_client.Recv()
          self._data_queue.put(msg)

          if buffer_counter < self._buffer_size - 1:
            buffer_counter += 1
          else:
            buffer_counter = 0
            self.has_data.emit()
      except socket.timeout:
        pass
    self._aio_client.Close()


def main():
  app = QtGui.QApplication(sys.argv)
  unused_win = PlotterMainWindow()
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()
