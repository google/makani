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

function [euler] = dcm_to_euler(dcm)
% dcm_to_euler -- converts Attitude rotation matrix to Euler angles.
% [euler] = dcm_to_euler(dcm)
%
% Arguments --
%
% dcm: [3 x 3 x no_timesteps] Attitude array
%
% Output Values --
%
% euler: [3 x no_timesteps] Euler angles (1-roll, 2-pitch, 3-yaw)
%
euler = zeros(1,3,length(dcm));

euler(1,1,:) = atan2(dcm(2,3,:), dcm(3,3,:));
euler(1,2,:) =-atan2(dcm(1,3,:), sqrt(1 - dcm(1,3,:).^2));
euler(1,3,:) = atan2(dcm(1,2,:), dcm(1,1,:));

euler = squeeze(euler);