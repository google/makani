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

function [] = plot_vec_of_signals(time, signal, legend_symbol, plot_mag, valid)
% plot_vec_of_signals -- Plots a set of signals for easy comparison.
%
% plot_vec_of_signals(time, signal, legend_symbol, plot_mag, valid)
%
% Example Usage:
%
% t = get_time(A, 'control_zero');
% mfig('c: Acc', 'Time [s]', 'Acc. [m/s^2]')
% plot_vec_of_signals(t, c.control_input.imus.acc, 'a', true)
%
% Arguments
%
% time: An array of length N containing sample times.
% signal: A structure with fields x, y, and z each of which are N-by-m
%     matrices with m <= 4.
% legend_symbol: String to be displayed in the legend.
% plot_mag: Indicates if the magnitude of each vector should also
%     be plotted (default is false).
% valid: A logical array of length N selecting the samples to be
%     plotted (default is all true).

if nargin < 4
  plot_mag = false;
end
if nargin < 5
  valid = repmat(true, size(time));
end

styles = {'-', '--', ':', '-.'};

signal_mag = sqrt(signal.x.^2 + signal.y.^2 + signal.z.^2);

m = size(signal.x, 1);
legend_entries = {};
for k = 1:m
  plot(time(valid), signal.x(k, valid), ['b' styles{k}], ...
       time(valid), signal.y(k, valid), ['g' styles{k}], ...
       time(valid), signal.z(k, valid), ['r' styles{k}]);
  legend_entries = horzcat(legend_entries, ...
                           {[legend_symbol '_{' num2str(k) 'x}'], ...
                            [legend_symbol '_{' num2str(k) 'y}'], ...
                            [legend_symbol '_{' num2str(k) 'z}']});
  if plot_mag
    plot(time(valid), signal_mag(k, valid), ['k' styles{k}]);
    legend_entries = horzcat(legend_entries, ...
                             {['|' legend_symbol '_{' num2str(k) '}|']});
  end
end
legend(legend_entries{:});
end
