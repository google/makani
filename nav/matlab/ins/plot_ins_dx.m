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

function [fig_i] = plot_ins_dx(out, t0, t1, fig_i)
if ~exist('t0','var')
  t0 = 0;
end
if ~exist('t1','var')
  t1 = inf;
end
if ~exist('fig_i','var')
  fig_i = 0;
end

p = out.param;
s = p.states;

ii = find(out.init > 0);
ii = ii(get_interval(out.t(ii), t0, t1));
t0 = max(t0, out.t(ii(1)));
t1 = min(t1, out.t(ii(end)));


fig_i = plot_dx(out, ii, fig_i, s, s.R_IB_E, out.r_ib_e, out.r_ib_e(ii(1), :));
fig_i = plot_dx(out, ii, fig_i, s, s.V_IB_E, out.v_ib_e);
fig_i = plot_dx(out, ii, fig_i, s, s.THETA_BI, out.theta_be);
fig_i = plot_dx(out, ii, fig_i, s, s.THETA_SA, out.mrp_sa * 4);
fig_i = plot_dx(out, ii, fig_i, s, s.THETA_AG, out.mrp_ag * 4);
fig_i = plot_dx(out, ii, fig_i, s, s.ACCEL_BIAS, out.b_a);
fig_i = plot_dx(out, ii, fig_i, s, s.ACCEL_WALK, out.c_a);
fig_i = plot_dx(out, ii, fig_i, s, s.ACCEL_SCALE, out.k_a);
fig_i = plot_dx(out, ii, fig_i, s, s.ACCEL_ORTHO, out.no_a);
fig_i = plot_dx(out, ii, fig_i, s, s.ACCEL_OFFSET_Y, out.r_ba_ay);
fig_i = plot_dx(out, ii, fig_i, s, s.ACCEL_OFFSET_Z, out.r_ba_az);
fig_i = plot_dx(out, ii, fig_i, s, s.GYRO_BIAS, out.b_g);
fig_i = plot_dx(out, ii, fig_i, s, s.GYRO_WALK, out.c_g);
fig_i = plot_dx(out, ii, fig_i, s, s.GYRO_SCALE, out.k_g);
fig_i = plot_dx(out, ii, fig_i, s, s.GYRO_ORTHO, out.no_g);
fig_i = plot_dx(out, ii, fig_i, s, s.GYRO_G_SENSE, out.F_g);
fig_i = plot_dx(out, ii, fig_i, s, s.WHEEL_RADIUS, out.radius);
fig_i = plot_dx(out, ii, fig_i, s, s.GPS_PR, out.gps_pr);
fig_i = plot_dx(out, ii, fig_i, s, s.CB_PHASE, out.cb_phase);
fig_i = plot_dx(out, ii, fig_i, s, s.CF_BIAS, out.cf_bias);
fig_i = plot_dx(out, ii, fig_i, s, s.CF_WALK, out.cf_walk);





function [fig_i] = plot_dx(out, ii, fig_i, s, s_indices, xhat, offset)
t0 = out.t(ii(1));
t1 = out.t(ii(end));
for k = 1:length(s_indices)
  ss = s_indices(k);

  if ismember(ss, s.select)
    fig_name = s.name{ss};
    fig_i = inc_figure(fig_i, fig_name);
    scale = s.scale(ss);
    sigma = out.sigma_xx(:, ss) * scale;
    escaped_name = strrep(s.symbol{ss}, '_', '\_');

    % Plot error state.
    h = zeros(2, 1);
    h(1) = subplot(2, 1, 1);
    hold on;
    plot(out.t(ii), sigma(ii), '-', out.t(ii), -sigma(ii), '-', ...
         'Color', [0.75, 0.75, 0.75]);
    line([t0; t1], [0; 0], 'Color', [0.9, 0.9, 0.9]);
    for mm = 1:length(out.innov)
      innov = out.innov{mm};
      if ~isempty(innov)
        jj = get_interval(innov.t, t0, t1);
        kk = jj(innov.valid(jj) ~= 0);
        plot(innov.t(kk), innov.dx_plus(kk, ss) * scale, '+');
        kk = jj(innov.valid(jj) == 0);
        plot(innov.t(kk), innov.dx_plus(kk, ss) * scale, 'o');
      end
    end
    hold off;
    axis tight;
    ylabel(sprintf('\\delta %s (%s)', escaped_name, s.unit{ss}));
    title(sprintf('%s error state', s.name{ss}));

    % Plot navigation state.
    h(2) = subplot(2, 1, 2);
    hold on;
    if ~exist('offset', 'var') || isempty(offset)
      delta = 0;
    elseif length(offset) == 1
      delta = offset;
    else
      delta = offset(k);
    end
    plot(out.t(ii), (xhat(ii, k) - ones(length(ii), 1) * delta) * scale, 'b');
    hold off;
    xlabel('Time (s)');
    if delta > 0
      ylabel(sprintf('%s - %g (%s)', escaped_name, delta, s.unit{ss}));
    elseif delta < 0
      ylabel(sprintf('%s + %g (%s)', escaped_name, -delta, s.unit{ss}));
    else
      ylabel(sprintf('%s (%s)', escaped_name, s.unit{ss}));
    end
    linkaxes(h, 'x');
  end
end
