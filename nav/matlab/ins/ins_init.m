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

function [ins] = ins_init(param, data, start_time, message_types)
% INS_INIT  Initialize default INS structure.
%
% ins = ins_init(param, data, start_time)
% Initialize default INS structure.
%
% param: Specify algorithm configuration parameters. Use ins_param to create
%     the default structure.
% data: A properly formatted data structure to process.
% start_time: Start time to initialize.
% message_types: Optional. Process a subset of the available data message
%     types. Specify an array of message types to process.
%
% Return values:
%
% ins: Final state of inertial navigation system. Use ins_parse to resume
%     processing from the last iteration.


ins = struct();
ins.param = param;
ins.message_types = message_types;
ins.iter = find(data.order.t >= start_time, 1) - 1;
start_time = data.order.t(ins.iter + 1);

% Set flag to initialize as soon as possible.
ins.reinit = 1;
ins.init = 0;  % Initialization count.

% Initialize circular buffers for all data types.
for i = 1:length(data.order.name)
  ins.(data.order.name{i}) = ins_cbuf_init(512);
end

ins.gps = init_gps(param, data, start_time);
ins.gps_1 = ins.gps;

ins.t_stationary = 0;
ins.t_wheel = 0;

% Initialize navigation state.
ins.xhat = ins_xhat_initialize(param, start_time);

% Initialize error state.
Pxx_minus = eye(param.states.count);
dx_minus = zeros(param.states.count, 1);
ins.dx = ins_dx_initialize(Pxx_minus, dx_minus, start_time, 0);





function [gps] = init_gps(param, data, start_time)
% Initialize GPS data structure.
gps = struct();
gps.iono = struct();  % Ionosphere corrections.
gps.obs = {};         % Recent observations.
gps.eph = {};         % Ephemerides.
gps.xhat = ins_xhat_initialize(param, start_time - 1);
gps.handle = [];
gps.tow = -1;
gps.Phi_0k = [];
gps.valid = zeros(32, 1);

[qc_ura, tc_ura] = ins_bi_model(2.0, param.gps.pr_bias_tau);
gps.qc_ura = ones(32, 1) * qc_ura;
gps.tc_ura = ones(32, 1) * tc_ura;

% Load initial GPS telemetry from RINEX when available.
if isfield(data, 'rinex')
  % Populate GPS ionosphere corrections with RINEX data.
  gps.iono = data.rinex.header.iono;

  return

  % Populate GPS ephemerides with RINEX data occurring before the dataset.
  ii = find(data.rinex.records.t_oe < data.eph.t_oe(1));
  for i = 1:length(ii)
    eph = get_message(data.rinex.records, ii(i));
    eph.t = start_time;
    eph.anti_spoof = 0;
    eph.alert = 0;
    gps.eph{eph.prn} = eph;
  end

  % Update GPS ephemerides with data occurring before the processing interval.
  ii = find(data.eph.t < start_time);
  for i = 1:length(ii)
    eph = get_message(data.eph, ii(i));
    gps.eph{eph.prn} = eph;
  end
end
