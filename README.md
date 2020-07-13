# Makani

Makani was a project to develop a commercial-scale airborne wind turbine, culminating in
a flight test of the Makani M600 off the coast of Norway. All Makani software has now been
open-sourced. This repository contains the working Makani flight simulator, controller (autopilot),
visualizer, and command center flight monitoring tools. Additionally, almost all avionics firmware
is also included, albeit potentially not in a buildable state, due to the removal of some
third-party proprietary code. We hope that this code will be inspirational and useful to the
kite-based windpower and wider communities.

## A brief tour of the code

 * `analysis` - Miscellaneous analysis scripts. The most important files here
   are `analysis/control/crosswind.py`, which generates the gains for the
   crosswind inner loop controller, and
   `analysis/control/generate_hover_controllers.m`, which generates the gains
   for the hover controller.
 * `avionics` - Firmware for the winch, ground station, motors, network, servos, batteries,
   network switches, GPS, strobe lights, etc.
 * `control` - The hover, transition-in, crosswind, and off-tether flight controllers.
 * `common`
 * `config` - A Python-based configuration system produces a JSON dict
   specifying all system, controller, and simulation parameters. This is
   translated into a read-only C structure at compile-time.
 * `database` - Aero tables, Pitot calibration tables, etc.
 * `documentation`
 * `vis` - OpenGL-based visualizer that depicts the state of the system during
   simulation and real flight.

## How to build

This code base was originally designed to run on Linux systems running the Debian Stretch
distribution. For the convenience of future users, we are shipping this open source release
with a script to create the necessary environment within Docker.

## How to read the logs

The Control Telemetry Users' guide, included as a PDF with this distribution, gives a full
description of log file data structures. Below are examples for how to load and plot log data
using Python or MATLAB.

### Python
Here's a small example showing how to load an h5 log file in Python and plot a variable (in this case, the kite's altitude). Accessing the log files is made much less painful by enabling tab-completion of telemetry fields; instructions are in `lib/python/ipython_completer.py`.
```
import h5py
import pylab

log = h5py.File('20161121-142912-flight01_crosswind.h5', 'r')
C = (log['messages']['kAioNodeControllerA']
        ['kMessageTypeControlTelemetry']['message'])
pylab.plot(C['time'], -C['state_est']['Xg']['z'])
pylab.show()`
```
Additionally, by starting Python with `bazel-bin/lib/bazel/pyembed ipython`, you will be able to access some Makani library functions (like DcmToAngle) directly from Python.

You can also explore the field names using `.items()` or `.keys()` and `.dtype` as appropriate:
```
log.keys()  # Shows [u'bad_packets', u'messages', u'parameters']
log[‘messages’].keys()  # Shows [u’kAioNodeBattA', … ]

# The number of messages of this type in the h5 file:
log['messages/kAioNodeBattA/kMessageTypeSlowStatus'].len()

# The first message:
log['messages/kAioNodeBattA/kMessageTypeSlowStatus'][0]

# The nested field names:
log['messages/kAioNodeBattA/kMessageTypeSlowStatus'][0].dtype`
```

### MATLAB
Here's a small example showing 3 ways to load an h5 log file in MATLAB and plot a variable (in this case, the kite's altitude).

#### Method 1 (quick to load a specific telemetry dataset to workspace with MATLAB built in function):
```
C = h5read('20161121-142912-flight01_crosswind.h5', '/messages/kAioNodeControllerA/kMessageTypeControlTelemetry');
time = C.message.time;
altitude = -C.message.state_est.Xg.z;
figure;
plot(time, altitude)
```

#### Method 2 (takes long to load the data to workspace but all telemetry data is accessible on load):
NOTE: Only works in MATLAB 2016a and earlier. Find out your MATLAB version by running “ver” on the console. You need the makani repository loaded to your computer (see ‘How to get the code’ section below).
Open MATLAB and navigate to the following directory on the console:
```$MAKANI_HOME/analysis```
Run the following script in the MATLAB console to set all relevant paths:
```SetMatlab```
Now you can run the following code on console to access telemetry data:
```
log = h5load('20161121-142912-flight01_crosswind.h5');
C = log('/messages/kAioNodeControllerA/kMessageTypeControlTelemetry');
Time = C.message.time;
Altitude = -C.message.state_est.Xg.z
figure;
plot(Time, Altitude)
```

#### Method 3 (the best of both worlds! Lazily loads all datasets quickly):
You need the makani repository loaded to your computer (see ‘How to get the code’ section below).
Open MATLAB and navigate to the following directory on the console:
```$MAKANI_HOME/analysis```
Run the following script in the MATLAB console to set all relevant paths:
```SetMatlab```
Now open the H5Plotter (a GUI interface for opening and plotting H5 log data) by running the following in the MATLAB console:
H5Plotter
Load a H5 log file using the ‘Choose’ button in the top right corner.
Once the file is loaded, datasets appear in ‘AIO Nodes’ panel box. Click on one or more of these nodes to access the corresponding datasets in the ‘AIO Messages’ panel box. Only datasets common to all selected AIO nodes are shown.
Once you have selected data to plot in the ‘AIO Messages’ panel box, use the ‘plot’ button at the bottom right corner to visualize the data. Holding ctrl or shift allows multiple fields to be selected and selecting a node in the tree will plot all data contained beneath that node. Data can also be exported by right clicking.
NOTE: You can plot multiple datasets simultaneously on the same time axes. How many datasets you can plot at the same time is only limited by your machine’s RAM; be judicious about this.

#### Matlab Example: Plot roll, pitch, and yaw.
Here's a small example that converts the `dcm_g2b` matrix into Euler angles.
```
% Read the log file.
filename = '20161121-142912-flight01_crosswind.h5';
C = h5read(filename, ...
    '/messages/kAioNodeControllerA/kMessageTypeControlTelemetry');

% Fetch the dcm_g2b matrix 
dcm_g2b = C.message.state_est.dcm_g2b.d;

% Transpose the dcm_g2b matrix.
dcm_g2b = permute(dcm_g2b, [2 1 3]);

% Compute Euler angles.
[yaw, pitch, roll] = dcm2angle(dcm_g2b, 'ZYX');

% Plot the results.
plot(C.message.time, roll  * 180/pi, '.', ...
     C.message.time, pitch * 180/pi, '.',...
     C.message.time, yaw   * 180/pi, '.')
legend('roll', 'pitch', 'yaw')

ylim([-180 180]);
set(gca, 'YTick', -180:30:180);
grid on;
xlabel('controller time [s]');
ylabel('angle [degrees]');
title(['flight attitude (' filename ')'], 'interpreter', 'none');
```
