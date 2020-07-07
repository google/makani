% Copyright 2020 Makani Technologies LLC
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

function [ loop_data ] = LoopAverager(loop_angles, time_data, bins, varargin)
% LoopAverager -- Function to get mean and standard deviations over all loops.
% [loop_data] = LoopAverager(loop_angles, time_data, n_bins)
% [loop_data] = LoopAverager(loop_angles, time_data, angles_bin, 'angle')
%
% LoopAverager will split apart the time-series data during crosswind into
% separate bins as well as compute the average and standard deviation of a
% common, interpolated angular position. Common angular positions is defined as
% the midpoint of the bin.
% If bins are specified as angle array rather than an integer, then these are
% used as common angular positions.
% Use TimeAverager.m for time based averaging e.g. net power calculations.
%
% Inputs:
% loop_angles: [1 x n] array; the crosswind angular location [rad]
% time_data:   [m x n] array; disambiguous data type;
%                             the data to be segregated on a per-loop basis
% bins:        [#] int; no of discrete histogram angle bins for the loop data
% bins:        [1 x n] array; user specified common angles [rad],
%                             followed by 'angle' specifier
%
% Outputs:
% loop_data:        struct; contains the per-loop segregated arrays of data and
%                           contains an independent array of the mean data and
%                           its standard deviation
%   .indx:          [int]; the integer number associated with the loop
%   .common_angles: [m x n_bins] array; the common bucket angles [rad]
%   .average:       [m x n_bins] array; the mean of the interpolated time_data
%   .stddev:        [m x n_bins] array; the standard deviation of the
%                                       interpolated time_data array
%

% second order forward difference, keeping consistent vector length, derivative starts at index 3
dth_di        = zeros(size(loop_angles));
dth_di(3:end) = diff(loop_angles,2);

% clear any slope changes, where the loop angle is not near 2*pi
% reject all peaks when loop angle is less than 95% of 2*pi
dth_di(find(loop_angles < 0.95*2*pi)) = 0;

% these are the first index for the data of the *current* loop
% second order difference helps spot change in curvature of loop angle plot
% which is more reliable for spotting peaks, irrespective of number of points.
endloop_indices = [];
for i = 1:length(dth_di)-2;
  if dth_di(i+1) < -1E-2 & dth_di(i+1) < dth_di(i) & dth_di(i+1) < dth_di(i+2);
    endloop_indices = [endloop_indices, i];
  end
end

% remove any double peaks i.e. threshold peak separation
false_id = find(diff(endloop_indices) < 10);
endloop_indices(false_id) = [];

nstruct_loops = length(endloop_indices) - 1;

if nstruct_loops > 0;
  % construct common angles
  if ~isempty(varargin) & strcmp(varargin{1}, 'angle')
    if size(bins, 1) > 1
      bins = transpose(bins);
    end
    common_angles = bins;
  else
    bins = round(bins);
    del_angle = 2*pi/bins;
    % common angles defined as midpoint of the bins
    common_angles = linspace(2*pi - del_angle/2, 0 + del_angle/2, bins);
  end

  % construct independent interpolation vector
  x_ind = [];
  angle_loop_id = [];
  for ii = 1:nstruct_loops
    x_ind = [x_ind, common_angles - ii*2*pi];
    angle_loop_id = [angle_loop_id, ones(size(common_angles))*ii];
  end
  clear ii;
  n_bins = length(common_angles);

  % unwrap the loop angle to use interpolation in one step
  loop_angles_unwraped = loop_angles;
  for i = 1:nstruct_loops
    % build field names cell array
    loopname{i} = ['Loop_', sprintf('%i',i)];

    jstart = endloop_indices(i);
    jend   = endloop_indices(i+1);
    % unwrap the loop angle to decrement values in each successive loop with 2*pi
    loop_angles_unwraped(jstart:jend) = loop_angles(jstart:jend) - i*2*pi;
  end
  loop_angles_unwraped(jend:end) = loop_angles(jend:end) - (i+1)*2*pi;
  clear i;

  % interpolate the time data
  time_data_interp = interp1(loop_angles_unwraped, time_data', x_ind);
  % interp1 forces x array to be a vector, the result needs to transposed if the
  % time data is more than one time series
  if size(time_data, 1) > 1
    time_data_interp = transpose(time_data_interp);
  end

  % build the return struct
  loop_data = struct();
  loop_data.average = zeros(size(time_data,1),n_bins);
  loop_data.common_angles = common_angles;

  % used to compile mean and standard deviation of interpolated values
  tmp_average = zeros(size(time_data,1), n_bins);
  for i=1:nstruct_loops
    % split the data into raw data bins
    jstart = endloop_indices(i);
    jend   = endloop_indices(i+1)-1;

    %compile your loop separated data
    loop_data.(loopname{i}).indx       = i;
    loop_data.(loopname{i}).angles     = loop_angles(jstart:jend);
    loop_data.(loopname{i}).raw        = time_data(:,jstart:jend);
    loop_data.(loopname{i}).interp_dat = time_data_interp(:, find(angle_loop_id == i));
    tmp_average = (tmp_average + loop_data.(loopname{i}).interp_dat);
  end

  % average & stddev
  loop_data.average = tmp_average / nstruct_loops;

  % compute stddev
  tmp_stddev = zeros(size(time_data,1), n_bins);
  if nstruct_loops > 1
    for i=1:nstruct_loops
      tmp_stddev = tmp_stddev + (loop_data.(loopname{i}).interp_dat - loop_data.average).^ 2 ;
    end
    loop_data.stddev = sqrt(tmp_stddev/(nstruct_loops - 1));
  else
    loop_data.stddev = 0;
  end
else
  loop_data = [];
end