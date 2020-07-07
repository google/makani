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

function [bjunc] = resample_bjunc(LCPB_dataset, C_dataset, info_dataset, varargin)
% resample_bjunc -- extract 'original' 10Hz measurements from the bridle
% junction load pin and interpolate to a new time vector. This removes the
% stairstep effect in current 100Hz wing data that would make direct
% comparsions very 'noisey' and cuase an effective ~0.05s offset
% [bjunc_tension] = resample_bjunc(LCPB_dataset, time, varargin)
%
% Arguments-
%
% LCPB_dataset: [struct] Loadcell PortB dataset from the controller h5 file
%               e.g. LCPB_dataset= h5read(h5_filename,'/messages/kAioNodeLoadcellPortB/kMessageTypeLoadcell');
% C_dataset: [struct] C_dataset= h5read(h5_filename,'/messages/kAioNodeControllerA/kMessageTypeControlDebug');
%
% Optional variables-
% info: [struct] ideally include h5 info structure to account for slight
%               time offset
%           e.g. info_dataset= h5read(h5_filename,'/info');
%
% Output Values-
%
% bjunc: [structure] has the resamples variables from the bjunc, including:
%       tension: [dbl col vector] resampled and synced tether tension from the bridle
%           junction load cell
%       roll: [dbl col vector] resapmled and synced roll encoder measurement from the bridle
%           junction load cell

interpmethod = 'linear';

if nargin == 3
    if isstruct(info_dataset)
        use_gps_correction = 1;
    else
        use_gps_correction = 0;
        warning('error using GPS correction, not applying correction')
    end
elseif nargin == 2
    use_gps_correction = 0;
elseif nargin > 3
    warning('no optional variables or flags set up yet')
end

if use_gps_correction
    time0 = get_time_zero(C_dataset, info_dataset);
    node_time_C = get_node_time(C_dataset, info_dataset, 'verbose','yes');
    time = node_time_C-time0;
    node_time_LCPB = get_node_time(LCPB_dataset, info_dataset, 'verbose','yes');
    time_LCPB = node_time_LCPB - time0;
else
    time0 = get_time_zero(C_dataset);
    node_time_C = get_node_time(C_dataset, 'verbose','yes');
    time = node_time_C-time0;
    node_time_LCPB = get_node_time(LCPB_dataset, 'verbose','yes');
    time_LCPB = node_time_LCPB - time0;
end

bjunc.time_LCPB = time_LCPB;
bjunc.tension_raw = double(LCPB_dataset.message.bridle_junc.junc_load);
bjunc.tension= interp1(time_LCPB(abs([0;diff(bjunc.tension_raw)])>0),bjunc.tension_raw(abs([0;diff(bjunc.tension_raw)])>0),time,interpmethod,0);
bjunc.roll = interp1(time_LCPB,double(LCPB_dataset.message.bridle_junc.junc_angle),time,interpmethod,0);

