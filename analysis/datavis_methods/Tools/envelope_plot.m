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

function [envelopes_out] = envelope_plot (xx, yy, envelopes, plot_info, export_info)
% envelope_plot -- plot convex envelopes for various cases and optionally save video.
% envelopes_out = envelope_plot (xx, yy, envelopes, plot_info, export_info)
%
% Arguements--
%
% xx:          [n x 1] Data to be plotted on x axis
% yy:          [n x 1] Data to be plotted on y axis
% envelopes:   [struct] Multiple envelopes with
%              envelopes.a.data  - (x,y) data for envelope 'a'
%              envelopes.a.color - display color name or RGB vector for 'a'
%              envelopes.a.label - label for envelope 'a'
%              envelopes.b       - data, color, label information for envelope 'b'
%              envelopes.c       - data, color, label information for envelope 'c'
% plot_info:   [struct] Preferences for plotting
%              .samples_per_plot - [int] data segment for length each frame
%              .samples_overlap  - [int] samples to retain from previous frame
%              .time             - [n x 1] time array, same size as xx, yy
%              .fig_no           - [int] specify a unique integer for figure handle
%              .xlabel           - [string] x axis label for plot
%              .ylabel           - [string] y axis label for plot
%              .title            - [string] title for plot and figure window
%              .identifiers      - [n x 1] used to differentiate data e.g. rpx cases
%              .identifiers_label- [string] e.g. 'case'
%              .identifiers_map  - [container] Value-label map for unique identifiers
%              .axis_style       - [string] e.g. 'normal', 'square', 'equal'
% export_info: [struct] Preferences for video saving
%              .save_plot        - [0, 1] Save .fig and .png files?
%              .save_movie       - [0, 1] Save video file?
%              .view_movie       - [0, 1] View animation? if save = 1, view defaults to 1
%              .file_name        - [string] Filename for video output e.g. loads_movie
%
% Return Values--
%
% envelopes_out: [struct] Envelope data for all data and sorted by identifiers

% parse the plot information
samples_per_plot = plot_info.samples_per_plot;
samples_overlap  = plot_info.samples_overlap;

ids              = plot_info.identifiers;
id_lbl           = plot_info.identifiers_label;
id_map           = plot_info.identifiers_map;

fig_no           = plot_info.fig_no;
xlbl             = plot_info.xlabel;
ylbl             = plot_info.ylabel;
titl             = plot_info.title;

tt               = plot_info.time;

if isfield(plot_info, 'axis_style')
  axis_style     = plot_info.axis_style;
else
  axis_style     = 'normal';
end

% if save_movie = 1, view_movie defaults to 1
if export_info.save_movie == 1
  export_info.view_movie = 1;
end

% find the unique identifiers and build a colormap
unique_cases = unique(ids);
% get distinct colors from a pallette
co_list = distinct_color(length(unique_cases));
% max(unique_cases) - min(unique_cases) + 1
co_map = containers.Map(unique_cases, num2cell(co_list,2));

% initialize axes limits
x_min = [];
x_max = [];
y_min = [];
y_max = [];

% plot limit envelopes
hh = figure(fig_no);
set(hh, 'Name', titl{1});
hold on; grid on; box on;
all_envelopes = fieldnames(envelopes);
for ii = 1:numel(all_envelopes);
  envelope_data      = getfield(getfield(envelopes, all_envelopes{ii}), 'data' );
  envelope_color     = getfield(getfield(envelopes, all_envelopes{ii}), 'color');
  envelope_label{ii} = getfield(getfield(envelopes, all_envelopes{ii}), 'label');
  plot (envelope_data(:,1), envelope_data(:,2), '--', 'color', envelope_color, 'linewidth', 4)
  x_min = min([envelope_data(:,1); x_min]);
  x_max = max([envelope_data(:,1); x_max]);
  y_min = min([envelope_data(:,2); y_min]);
  y_max = max([envelope_data(:,2); y_max]);
end
% adjust plot limits with 5% margin
delta_x = abs(x_max - x_min)*0.05;
delta_y = abs(y_max - y_min)*0.05;
x_min = x_min - delta_x;
x_max = x_max + delta_x;
y_min = y_min - delta_y;
y_max = y_max + delta_y;

xlim ([x_min, x_max])
ylim ([y_min, y_max])

xlabel (['\bf' xlbl]);
ylabel (['\bf' ylbl]);
title (titl);
axis (axis_style);
legend (envelope_label, 'location', 'northeast', 'orientation','horizontal', ...
                        'FontSize', 12,'FontWeight', 'Bold', 'AutoUpdate','off');
clear all_envelopes ii;

% add colorbar to identify cases
cc = colorbar;
colormap(co_list);
caxis([min(ids) max(ids)])
title(cc, ['\bf' id_lbl]);

% add labels to the colorbar
for ii = 1:length(unique_cases)
  bar_label{ii} = id_map(unique_cases(ii));
end
clear ii;
set(cc,'YTickLabel',bar_label)
set(cc,'YTick',linspace(min(unique_cases)+0.5, max(unique_cases)-0.5, length(unique_cases)));
set(cc,'YTickLabel',bar_label)
set(cc, 'FontSize', 10, 'FontWeight', 'bold')

% initialize video saving
if export_info.save_movie == 1;
  vid = VideoWriter(export_info.file_name);
  vid.Quality = 100;
  vid.FrameRate = 60;
  open(vid);
end

% initialize plotting parameters
num_points = length(xx);
num_plotted = 0;

% initialize polygon and plot library
p2 = []; % data plot
polygon_lib = struct();
for ii = 1:length(unique_cases);
  polygon_lib.([id_lbl, num2str(unique_cases(ii))]) = [];
  p1.([id_lbl, num2str(unique_cases(ii))])          = [];
end
clear ii;

% sort the samples by identifiers to process faster if no save_movie is false
if export_info.view_movie == 0;
  case_change = find(diff(ids) ~= 0);
  case_change = [0; case_change; length(ids)];
  samples_step = diff(case_change);
  samples_overlap = 0;
else
  samples_step = samples_per_plot;
end
clear case_change;
ii = 1;

while num_points > num_plotted + samples_step(ii);
  % if less than 10 samples in a case, advance to next case
  % this is to meet minimum samples requirements for convhull calculations
  if samples_step(ii) < 10;
    num_plotted = num_plotted + samples_step(ii);
    ii = ii + 1;
  end
  % create data segment based on plot_info preferences
  idx_to_plot = [num_plotted + 1 : num_plotted + samples_step(ii) - 1];

  % check if multiple identifiers exist in idx_to_plot
  if export_info.view_movie == 1
    case_change = find(diff(ids(idx_to_plot)) ~= 0);
    if ~isempty(case_change);
      case_change = case_change(1);
      samples_overlap = length(idx_to_plot(case_change:end));
      idx_to_plot = idx_to_plot(1:case_change);
    else
      samples_overlap = plot_info.samples_overlap;
    end
    clear case_change;
  end

  % identification based on data segment
  plot_id = max(ids(idx_to_plot));
  time_id = int32(mean(tt(idx_to_plot)));

  % find the convex envelope of the data segment
  k = convhull(xx(idx_to_plot), yy(idx_to_plot));
  new_polygon = [xx(idx_to_plot(k)), yy(idx_to_plot(k))];

  % append any existing case specific envelope information in the library
  new_polygon = [polygon_lib.([id_lbl, num2str(plot_id)]); new_polygon];

  % find new boundary, update the library and plotting data
  polygon_lib.([id_lbl, num2str(plot_id)]) = new_polygon(boundary(new_polygon,0),:);
  plot_polygon = polygon_lib.([id_lbl, num2str(plot_id)]);

  % delete past envelope, plot from updated library
  delete(p1.([id_lbl, num2str(plot_id)]));
  p1.([id_lbl, num2str(plot_id)]) = fill (plot_polygon(:,1), plot_polygon(:,2), co_map(plot_id));
  set(p1.([id_lbl, num2str(plot_id)]),'facealpha',.25 ,'edgecolor',co_map(plot_id), ...
                                  'linewidth', 4, 'DisplayName', id_map(plot_id))

  % delete past data plot, overwrite from current segement
  delete(p2);
  p2 = plot (xx(idx_to_plot), yy(idx_to_plot), '.k');
  set(p2, 'DisplayName', 'Current Loop')
  title ([titl; sprintf('t = %.0f s, %s %.0f', time_id, id_lbl, plot_id)]);

  % this is to draw the current plot and
  % prepare to grab frame if video save is true
  if export_info.view_movie == 1;
    drawnow;
  end

  % save video from figure frame
  if export_info.save_movie == 1;
    writeVideo(vid, getframe(hh));
  end

  % update num_plotted, rollback overlap
  num_plotted = num_plotted + samples_step(ii) - samples_overlap;

  if export_info.view_movie == 0 & ii < length(samples_step);
    ii = ii + 1;
  end

end
clear ii;

% update envelope_out and add a global envelope for complete dataset
envelopes_out = polygon_lib;
all_envelopes = fieldnames(envelopes_out);
all_cases = [];

% concatenate all envelopes and find a global envelope
for ii = 1:numel(all_envelopes);
  all_cases = [all_cases; getfield(envelopes_out, all_envelopes{ii})];
end
clear all_envelopes ii;
all_cases = all_cases(boundary(all_cases, 0),:);

% append global envelope to envelopes_out
envelopes_out.global = all_cases;

% adjust plot axes limits if global boundary is outside plot limit
x_min = floor(min([all_cases(:,1); x_min]));
x_max = ceil (max([all_cases(:,1); x_max]));
y_min = floor(min([all_cases(:,2); y_min]));
y_max = ceil (max([all_cases(:,2); y_max]));
xlim ([x_min, x_max]);
ylim ([y_min, y_max]);
% find new axes limits
x_ticks = unique([x_min, get(gca,'XTick'), x_max]);
y_ticks = unique([y_min, get(gca,'YTick'), y_max]);

% delete data segment, and plot global envelope.
delete(p2);
p1 = fill(all_cases(:,1), all_cases(:,2), 'k');
set(p1, 'facealpha',0 ,'edgecolor', 'black', 'linewidth', 4, 'DisplayName', 'Global', 'linestyle', '--');
set(gca, 'XTick', x_ticks, 'YTick', y_ticks);
title ([titl])
hold off

% save video from figure frames
if export_info.save_movie == 1;
  writeVideo(vid, getframe(hh));
  close(vid);
end

% save .fig and .png
if export_info.save_plot == 1
  savefig(hh, [export_info.file_name, '.fig'],'compact');
  saveas (hh, [export_info.file_name, '.png']);
end