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

function [fig_i] = plot_ins_dz(data, t0, t1, fig_i)
if ~exist('t0','var') || isempty(t0)
  t0 = 0;
end
if ~exist('t1','var') || isempty(t1)
  t1 = Inf;
end
if ~exist('fig_i','var') || isempty(fig_i)
  fig_i = 0;
end

ii = find(data.init > 0);
t0 = data.t(ii(1));
t1 = data.t(ii(end));

for mm = 1:length(data.innov)
  if ~isempty(data.innov{mm})
    fig_i = plot_innov(data.param, data.innov{mm}, mm, t0, t1, fig_i);
  end
end


function [fig_i] = plot_innov(p, innov, mm, t0, t1, fig_i)
m = p.meas;
ii = get_interval(innov.t, t0, t1);
if ~isempty(ii)
  fig_name = m.name{mm};
  fig_i = inc_figure(fig_i, fig_name);
  scale = m.scale(mm);
  sigma = sqrt(innov.Pzz(ii, :)) * scale;
  dz = innov.dz(ii, :) * scale;
  t = innov.t(ii);
  escaped_name = strrep(m.symbol{mm}, '_', '\_');

  h = zeros(2, 1);
  h(1) = subplot(1, 10, 1:9);
  hold on;
  line([innov.t(ii(1)); innov.t(ii(end))], [0; 0], 'Color', [0.9, 0.9, 0.9]);
  plot(t, 1*sigma, '-', t, -1*sigma, '-', 'Color', [0.75, 0.75, 0.75]);
  plot(t, 2*sigma, '-', t, -2*sigma, '-', 'Color', [0.5, 0.5, 0.5]);
  plot(t, 3*sigma, '-', t, -3*sigma, '-', 'Color', [0, 0, 0]);
  plot(t, innov.dz_avg(ii, :), 'Color', 'm');

  jj = find(innov.valid(ii) ~= 0);
  plot(t(jj), dz(jj, :), 'b+');
  jj = find(innov.valid(ii) == 0);
  plot(t(jj), dz(jj, :), 'ro');
  hold off;
  axis tight;
  title(sprintf('%s innovation', m.name{mm}));
  xlabel('Time (sec)');
  ylabel(sprintf('%s (%s)', escaped_name, m.unit{mm}));

  h(2) = subplot(1, 10, 10);
  hold on;

  % Create histogram.
  sig = mean(mean(sigma, 1), 2);
  n_bins = 50;
  max_counts = 1;
  range = max(max(max(dz, [], 1), [], 2) - min(min(dz, [], 1), [], 2), sig);

  % Blue histogram (valid).
  bb = find(innov.valid(ii) ~= 0);
  b_dz = reshape(dz(bb, :), prod(size(dz(bb, :))), 1);
  b_bins = ceil(n_bins * (max(b_dz) - min(b_dz)) / range);
  [b_count, b_bin] = hist(b_dz, b_bins);
  if ~isempty(b_count)
    max_counts = max(max_counts, max(b_count));
  end

  % Red histogram (invalid).
  rr = find(innov.valid(ii) == 0);
  r_dz = reshape(dz(rr, :), prod(size(dz(rr, :))), 1);
  r_bins = ceil(n_bins * (max(r_dz) - min(r_dz)) / range);
  [r_count, r_bin] = hist(r_dz, r_bins);
  if ~isempty(r_count)
    max_counts = max(max_counts, max(r_count));
  end

  barh(b_bin, b_count/max_counts, 1, 'FaceColor', 'b', 'EdgeColor', 'b');
  barh(r_bin, r_count/max_counts, 1, 'FaceColor', 'r', 'EdgeColor', 'r');

  % Normal distribution curve.
  x = linspace(-3*sig, 3*sig);
  y = exp(-x.^2 / (2 * sig^2));
  plot(y, x, 'k');
  hold off;
  set(h(2), 'XTick', [], 'YTickLabel', []);
  a = axis(h(1));
  axis([0, 1, a(3:4)]);

  linkaxes(h, 'y');
end
