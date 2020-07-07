# Initialization and data import

This will initialize matplotlib appropriately, import suitable modules,
and load your data. Note that `'/home/<your_username>/makani/logs/last.h5'`
is just an example log path, and an incomplete one, as `<your_username>` must
be replaced.

Also, if you are importing a ground log, you will need to replace
`'ControlDebug'` with `'ControlTelemetry'`.

```python
# We require a Qt backend for its window management API. You may have to install
# a new package to support this.
import matplotlib
matplotlib.use('Qt4Agg')

import h5py
import numpy as np
from matplotlib import pyplot

from makani.analysis.plot.python import mplot
from makani.analysis.plot.python import plot_groups

# Replace path_to_data_file with a string containing the path to your log file.
data = mplot.CollatedH5Data(
    ['/home/<your_username>/makani/logs/last.h5'],
    [('ControllerA', 'ControlDebug'),
     ('Simulator', 'SimTelemetry')])
```

# Examples

The examples below assume that the above initialization block has been run.

Note that it is useful to run
```python
mplot = reload(mplot)
plot_groups = reload(plot_groups)
```
if you are making changes to the plotting library while using it.

## Example: Trans-in Longitudinal

```python
pyplot.close('all')

c = plot_groups.ControllerPlots(data)
c.trans_in.PlotAeroAngles()
c.trans_in.PlotBodyRates()
c.trans_in.PlotAirspeed()

c.trans_in.PlotRadialPosition()
c.trans_in.PlotRadialVelocity()
c.trans_in.PlotAeroClimbAngle()

c.trans_in.PlotEulerAngleError()
c.trans_in.PlotAttitudeError()

c.trans_in.PlotElevator()
c.control_output.PlotMoment()
c.control_output.PlotThrust()
c.control_output.PlotMotors()
c.trans_in.PlotTension()

c.estimator.PlotVelocity(plot_glas=False)

c.planner.PlotFlightMode()
c.control_input.PlotGsg()
c.estimator.PlotTension()
c.control_input.PlotLoadcells()

c.trans_in.PlotTension()

c.trans_in.SetXlimTransIn()
```

## Example: Trans-in Lateral

```python
pyplot.close('all')

c = plot_groups.ControllerPlots(data)
c.trans_in.PlotAeroAngles()
c.trans_in.PlotBodyRates()
c.trans_in.PlotAirspeed()

c.trans_in.PlotLateralVelocity()
c.trans_in.PlotLateralPosition()

c.trans_in.PlotEulerAngleError()
c.trans_in.PlotAttitudeError()

c.trans_in.PlotAilerons()
c.trans_in.PlotRudder()
c.control_output.PlotMoment()
c.control_input.PlotGsg()

pyplot.plot(c.c['time'], c.c['trans_in']['int_roll'])
c.trans_in.SetXlimTransIn()
c.trans_in.PlotGates()
c.planner.PlotFlightMode()
```

## Example: Hover
```python
pyplot.close('all')

c = plot_groups.ControllerPlots(data)
c.hover.PlotHoverAngles()
c.hover.PlotHoverIntAngles()
c.hover.PlotHoverIntMoment()
c.hover.PlotHoverPosition()
c.hover.PlotHoverVelocity()
c.estimator.PlotTension()
c.estimator.PlotVelocity()
```

## Example: Estimator

```python
pyplot.close('all')

c = plot_groups.ControllerPlots(data)

c.estimator.PlotAccelerometer()
c.estimator.PlotGyros()
c.estimator.PlotMagnetometer()

c.estimator.PlotGpsReceiver()

c.estimator.PlotAttitudeError()
c.estimator.PlotGyroBiases()

c.estimator.PlotVelocity()
c.estimator.PlotPosition()


c.estimator.PlotVelocitySigmas()
c.estimator.PlotPositionSigmas()

c.estimator.PlotWindSpeed()
```

## Example: Compare last.h5 to last_z1.h5

```python
data1 = mplot.CollatedH5Data(
    ['/home/<your_username>/makani/logs/last.h5'],
    [('ControllerA', 'ControlDebug'),
     ('Simulator', 'SimTelemetry')])

data2 = mplot.CollatedH5Data(
    ['/home/<your_username>/makani/logs/last_z1.h5'],
    [('ControllerA', 'ControlDebug'),
     ('Simulator', 'SimTelemetry')])

pyplot.close('all')

p = mplot.PlotGroup()
c1 = plot_groups.ControllerPlots(data1, parent=p)
c2 = plot_groups.ControllerPlots(data2, parent=p)

c1.trans_in.PlotAeroClimbAngle()
c1.trans_in.SetXlimTransIn()
c2.trans_in.PlotAeroClimbAngle()
c1.trans_in.SetXlimTransIn()
```

## Example: Plot carrier to noise density ratio

```python
data = mplot.CollatedH5Data(
    ['/home/<your_username>/makani/logs/last.h5'],
    [('FcA', 'NovAtelObservations'),
     ('FcB', 'NovAtelObservations')])

pyplot.close('all')

a = plot_groups.AvionicsPlots(data)
a.gps.PlotCarrierToNoiseDensityRatioFcA()
a.gps.PlotCarrierToNoiseDensityRatioFcB()
```

## Example: Plot idle time

```python
data = mplot.CollatedH5Data(
    ['/home/<your_username>/makani/logs/last.h5'],
    [('FcA', 'NovAtelSolution'),
     ('FcB', 'NovAtelSolution')])

pyplot.close('all')

a = plot_groups.AvionicsPlots(data)
a.gps.PlotIdleTime()
```