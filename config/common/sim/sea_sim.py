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

"""Offshore sea simulator parameters."""

from makani.config import mconfig
import numpy as np


@mconfig.Config
def MakeParams():

  # Parameters for the sea model, based on the JONSWAP model.

  return {
      # See derivation in:
      # drive.google.com/open?id=10--gfN41OCA1k3tjxwAdSkv7BPQm0VIaORuc5npkUWk
      'use_waves': True,  # Flag to enable/disable waves.
      'significant_height': 1.5,  # Significant wave height [m].
      'peak_period': 6.5,  # Peak wave period [s].
      'gamma': -1.0,  # JONSWAP peak shape parameter [-]. A negative number
                      # forces the computation of gamma based on the values of
                      # significant_height and peak_period.
      'water_depth': 220.0,  # Water depth from mean sea level to sea bed [m].
                             # From 18041-000-01 Metocean Design Basis report.
      'number_of_waves': 50,  # Number of spectrum discretizations.
      'initial_frequency': 0.5,  # Initial discretized frequency [rad/s]. This
                                 # value should be about half of the peak
                                 # frequency (= 2*pi/peak_period) to ensure
                                 # that the spectrum is correctly sampled.
      'cutoff_frequency': 3.0,  # Final discretized frequency [rad/s].
                                # This frequency should be high enough to cover
                                # the area of the spectrum with significant
                                # amplitudes, and is driven by peak_period.
      'initial_frequency_sampling_delta': 0.02,  # Initial delta frequency for
                                                 # spectrum sampling [rad/s].
      'wave_heading_ned': np.radians(20.0+180.),  # Wave heading, clockwise from
                                                  # North [rad]. This is the
                                                  # direction the waves are
                                                  # traveling toward.
      'water_density': 1025.0,  # Water density [kg/m^3].
      'grid_half_length': 500.0,  # Half length of the grid for sea
                                  # visualization [m].
  }
