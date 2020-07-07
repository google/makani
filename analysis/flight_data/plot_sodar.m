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

function plot_sodar(filename)
% PLOT_SODAR Load and plot a comma-separated-value SODAR data file.
%
% plot_sodar('rpx03_15min_sodar.csv')
%
% The SODAR data may be retrieved from the Somoma Tech FTP site:
%
% Choose the "1Minute" or "15Minute" directory to get 1-minute or 15-minute
% averages, respectively.  The 1-minute data is very noisy, so the
% 15-minute data is typically more useful.
%
% Each CSV file on the FTP site (somewhat infuriatingly) contains only one
% row of data. To use this script you must merge these CSV files
% into one CSV file, deleting the header information from all but the first
% file. A unix shell pipeline to accomplish this is given below.
%
% You will most likely want to script the download. This can be done easily
% with wget. For example:
%
%   wget --password='REDACTED' --user=redacted \
%        ftp://redacted@ftp.redacted.com/Outgoing/SODAR/1Minute/\
%        ASP25002_161214{16..17}{00..60}.csv
%
% where your shell will do string expansion on {xx..yy} to create the
% sequence of filenames to download. (You may also find that the CSV files
% that you seek have been archived in a .zip file. Browse with a web
% browser to see what files exist. You may have to wait a few minutes for
% the file listing to fully populate.)
%
% Next, create a merged CSV file. Here's a fun UNIX command line to do the
% trick:
%
%   (head -q -n 1 A*.csv | head -n 1 ; tail -q -n +2 A*.csv) > merged.csv
%
% ...assuming the retrieved CSV files start with "A". Finally, pass the
% name of that merged CSV file to this script.

sodar = readtable(filename);
n_rows = size(sodar, 1);
rows = 1:n_rows;

% TODO: The altitude should be inferred from the fields in the CSV
% file; instead we just hardcode them here.
altitudes = [3, 50:10:500];

%% Plot the "column" plots: speed and direction as a function of altitude.
figure();
set(gcf, 'DefaultAxesColorOrder', jet(n_rows));

% Plot wind speed (ws).
subplot(1, 2, 1)

fields = cellfun(@(alt) sprintf('ws%d', alt), ...
    num2cell(altitudes), 'UniformOutput', false);

values = table2array(sodar(rows, fields));

plot(values, altitudes, '.-');
grid on
xlabel('wind speed [m/s]');
ylabel('altitude [m AGL]');
title('Wind speed');
xlim([0, 15])

% Plot wind direction (wd).
subplot(1, 2, 2);

fields = cellfun(@(alt) sprintf('wd%d', alt), ...
    num2cell(altitudes), 'UniformOutput', false);

values = table2array(sodar(rows, fields));

plot(values, altitudes, '.-');
grid on
xlabel('wind direction [deg]');
ylabel('altitude [m AGL]');
set(gca, 'Xtick', 0:45:360)
xlim([0,360])
title('Wind direction');
legend(table2cell(sodar(rows, 'time')), 'Location', 'Best');

%% Now plot SODAR data vs time.
figure();
n_altitudes = length(altitudes);
set(gcf, 'DefaultAxesColorOrder', jet(n_altitudes));

% Concatenate the date and time fields and form a Matlab datenum.
date_strs = strcat(datestr(table2array(sodar(:, 'date'))), ' ,', ...
                           table2array(sodar(:, 'time')));
t = datenum(date_strs);

subplot(2, 1, 1);
fields = cellfun(@(alt) sprintf('ws%d', alt), ...
    num2cell(altitudes), 'UniformOutput', false);

values = table2array(sodar(rows, fields));
plot(t, values, '.-');
grid on;
ylabel('speed [m/s]')
datetick('x');

subplot(2, 1, 2);
fields = cellfun(@(alt) sprintf('wd%d', alt), ...
    num2cell(altitudes), 'UniformOutput', false);

values = table2array(sodar(rows, fields));
plot(t, values, '.-');
set(gca, 'ytick', 0:45:360);
ylim([0, 360]);
grid on;
xlabel('time')
ylabel('direction [degrees]')
datetick('x');
end

