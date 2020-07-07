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

function [ avg_data ] = TimeAverager(loop_angles, time_data)
% TimeAverager -- Function to get time average and standard deviations over all loops.
% [avg_data] = TimeAverager(loop_angles, time_data)
%
% TimeAverager will perform a time average of the inputs, after separating the
% data into loops. Recommended to use this to calculate net power in the loop.
%
% Inputs:
% loop_angles: [1 x n] array; the crosswind angular location [rad]
% time_data:   [m x n] array; disambiguous data type;
%                             the data to be time averaged on a per-loop basis
%
% Outputs:
% avg_data:         struct; contains time averaged data and
%                           its per-loop based standard deviation
%   .average:       [m x 1] array; mean of complete time_data
%   .stddev:        [m x 1] array; the standard deviation of complete time_data
%   .Loop_<#>:      struct; contains loop-basis data
%       .indx:      [int] Loop identifier
%       .angles     [m x ..] angles in the loop
%       .raw        [m x ..] time data in the loop
%       .average:   [m x 1 ] time average data of this loop
%
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
    endloop_indices = [endloop_indices, i+1];
  end
end

% remove any double peaks i.e. threshold peak separation
false_id = find(diff(endloop_indices) < 10);
endloop_indices(false_id) = [];

nstruct_loops = length(endloop_indices) - 1;

if nstruct_loops > 0;
  % build the return struct
  avg_data = struct();
  avg_data.average = zeros(size(time_data,1), 1);
  avg_data.stddev = zeros(size(time_data,1), 1);

  % used to compile mean and standard deviation of interpolated values
  time_average = zeros(size(time_data,1), 1);
  for i=1:nstruct_loops
    % build field names cell array
    loopname{i} = ['Loop_', sprintf('%i',i)];

    % split the data into raw data bins
    jstart = endloop_indices(i);
    jend   = endloop_indices(i+1)-1;

    %compile your loop separated data
    avg_data.(loopname{i}).indx       = i;
    avg_data.(loopname{i}).angles     = loop_angles(jstart:jend);
    avg_data.(loopname{i}).raw        = time_data(:,jstart:jend);
    avg_data.(loopname{i}).average    = mean(time_data(:,jstart:jend), 2);
    time_average = time_average + avg_data.(loopname{i}).average;
  end

  % average & stddev
  avg_data.average = time_average / nstruct_loops;

  % compute stddev
  stddev = zeros(size(time_data,1), 1);
  for i=1:nstruct_loops
    stddev = stddev + (avg_data.(loopname{i}).average - avg_data.average) .^ 2 ;
  end
  avg_data.stddev= sqrt(stddev) / nstruct_loops;
else
  avg_data = [];
end