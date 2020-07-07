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

"""Utility functions for system identification."""

import matplotlib
import matplotlib.mlab as mlab
import numpy as np
import pylab


def EstimateTransferFunction(x, y, *args, **kwargs):
  """Estimate the transfer function from x to y using Welch's method."""
  pxy, f = mlab.csd(x, y, *args, **kwargs)
  pxx, _ = mlab.psd(x, *args, **kwargs)
  return pxy / pxx, f


def AddTransferFunctionToPlot(axes, frequencies, transfer_function_data,
                              coherence, style, label, do_unwrap=False,
                              min_coherence=0.0):
  """Add transfer function data to the given set of axes.

  Args:
    axes: Array of three axes, into which to plot the transfer
    function magnitude, phase, and coherence, respectively.
    frequencies: Array of frequencies at which the transfer function
        was measured.
    transfer_function_data: Array of complex-valued transfer function
        measurements.
    coherence: Array of coherence measurements.
    style: Style-string to be passed to Matplotlib.
    label: Descriptive string to include in plot legend.
    do_unwrap: Whether phase should be unwrapped.
    min_coherence: Minimum coherence required to show point in plot.
  """
  def Db(x):
    return 20.0 * np.log10(np.abs(x))
  def AngleDeg(x):
    phase = np.angle(x)
    if do_unwrap:
      phase = np.unwrap(phase)
    return np.rad2deg(phase)

  i = coherence > min_coherence
  axes[0].semilogx(frequencies[i], Db(transfer_function_data[i]), style)
  if len(axes) > 1:
    axes[1].semilogx(frequencies[i], AngleDeg(transfer_function_data[i]), style)
  if len(axes) > 2:
    axes[2].semilogx(frequencies, coherence, style, label=label)


def MakeAxes():
  axes = [None, None, None]
  axes[0] = pylab.subplot2grid((5, 1), (0, 0), rowspan=2)
  axes[1] = pylab.subplot2grid((5, 1), (2, 0), rowspan=2, sharex=axes[0])
  axes[2] = pylab.subplot2grid((5, 1), (4, 0), sharex=axes[0])
  return axes


def SetAxes(axes):
  """Set the default axes properties for a Bode plot."""
  ax = axes[0]
  ax.grid(b=True, which='major', linestyle='solid', color='gray')
  ax.grid(b=True, which='minor', linestyle='solid', color='gray')
  ax.set_ylim([-20.0, 3.0])
  ax.set_yticks(np.arange(-18.0, 3.0, 3.0))
  ax.set_ylabel('Magnitude [dB]')
  ax.set_title('Transfer function: commanded value to measured value.')

  ax = axes[1]
  ax.grid(b=True, which='major', linestyle='solid', color='gray')
  ax.grid(b=True, which='minor', linestyle='solid', color='gray')
  ax.set_ylim([-135.0, 45.0])
  ax.set_yticks(np.arange(-135.0, 45.0, 15.0))
  ax.set_ylabel('Phase [deg]')

  ax = axes[2]
  ax.set_ylabel('Coherence [#]')
  ax.set_xlabel('Frequency [Hz]')
  ax.grid(b=True, which='major', linestyle='solid', color='gray')
  ax.grid(b=True, which='minor', linestyle='solid', color='gray')

  for ax in axes:
    ax.set_xlim([0.09, 50.0])
    ax.set_xticks([0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0])
    ax.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())


def PlotTransferFunction(cmd_array, val_array, labels, sample_rate, axes=None,
                         settling_time=0.0, shutdown_time=0.0, nfft=128,
                         detrend='mean', do_unwrap=False, min_coherence=0.0):

  """Compute and plot transfer functions between two arrays of time series.

  Args:
    cmd_array: Array of command (input) values.  May be multi-dimensional.
    val_array: Array of measured (output) values.  Same dimensions as cmd_array.
    labels: Array of strings to use as legend labels.
    sample_rate: Sample rate in Hz.
    axes: Array of three axes for plotting, or None to create axes.
    settling_time: Duration of start of timeseries to ignore.
    shutdown_time: Duration of end of timeseries to ignore.
    nfft: Number of points at which to compute the Fourier transform.
    detrend: Detrend method: 'none', 'mean', or 'linear'.
    do_unwrap: Boolean specifying whether phase should be unwrapped.
    min_coherence: Minimum coherence required to include data points in plots.

  Returns:
    Array of axes containing resulting plots.
    Vector of frequencies at which transfer function was computed.
    Transfer function data.
    Coherence data.
  """

  assert cmd_array.shape == val_array.shape
  assert cmd_array.shape[1] == len(labels)
  assert sample_rate > 0.0

  if axes is None:
    axes = MakeAxes()

  # Common keyword arguments to Welch's method functions.
  psd_kwargs = {'Fs': sample_rate,
                'NFFT': nfft,
                'detrend': {'none': mlab.detrend_none,
                            'mean': mlab.detrend_mean,
                            'linear': mlab.detrend_linear}[detrend],
                'window': mlab.window_hanning}

  for i in range(cmd_array.shape[1]):
    settling_samples = np.round(sample_rate * settling_time)
    shutdown_samples = np.round(sample_rate * shutdown_time)
    assert settling_samples + shutdown_samples < val_array.shape[0]
    selected = slice(settling_samples if settling_samples > 0 else None,
                     -shutdown_samples if shutdown_samples > 0 else None)
    val = np.array(val_array[selected, i])
    cmd = np.array(cmd_array[selected, i])

    txy, f = EstimateTransferFunction(cmd, val, **psd_kwargs)
    coh, _ = mlab.cohere(cmd, val, **psd_kwargs)

    AddTransferFunctionToPlot(axes, f, txy, coh, '-', labels[i],
                              do_unwrap=do_unwrap,
                              min_coherence=min_coherence)

  return axes, f, txy, coh
