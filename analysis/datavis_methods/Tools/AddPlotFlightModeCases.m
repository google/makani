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

% AddPlotFlightModeCases function to add text markers for mode and case start times to the current plot
% Usage:
%     figure('name','tether tension')
%     plot(d.time,d.tether_tension)
%     AddPlotFlightModeCases(d.flight_modes,d.crosswind_cases)
%     grid
%     title([d.title,':  Unfiltered Tether Tension'])
%     xlabel('time (sec)')
%     ylabel('tension (N)')

% Description:
%   feed this the outputs from the get_modes_and_cases, and call after
%   making a plot to add labels for where modes and cases were switched
%
% Inputs:
%   flight_modes:       from get_modes_and_cases
%   crosswind_cases:    from get_modes_and_cases
%
% Outputs:
%   none
%
% Future Work:
%   - ...

function AddPlotFlightModeCases(flight_modes,crosswind_cases)

% Get the flight mode labels, specify rpx # for flights before rpx-08.
[ids, labels] = get_flight_mode_labels;

for ii = 1:size(ids, 1)
    flight_mode_def{ii, 1} = ['  ', num2str(ids(ii)), ': ' labels{ii}];
end
clear ids labels ii;

plot_axis= axis;
hold on
if ~isempty(crosswind_cases)
    plot(flight_modes.Tstart,ones(size(flight_modes.Tstart))*plot_axis(3),'kv',crosswind_cases.Tstart,ones(size(crosswind_cases.Tstart))*plot_axis(3),'kv')
    text(flight_modes.Tstart,ones(size(flight_modes.Tstart))*plot_axis(3),flight_modes.all_modes_names,'rotation',90,'FontSize',8)
    text(crosswind_cases.Tstart,ones(size(crosswind_cases.Tstart))*plot_axis(3),crosswind_cases.all_cases,'rotation',90,'FontSize',8)
else
    if isfield(flight_modes,'Tstart')
        plot(flight_modes.Tstart,ones(size(flight_modes.Tstart))*plot_axis(3),'kv')
        text(flight_modes.Tstart,ones(size(flight_modes.Tstart))*plot_axis(3),flight_modes.all_modes_names,'rotation',90,'FontSize',8)
    end
end
hold off
end