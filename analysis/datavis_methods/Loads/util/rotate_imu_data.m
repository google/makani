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

function [imu_data_rotated] = rotate_imu_data (imu_message, rotation_angles, invert_rotation)
% rotate_imu_data -- implements anglular rotation on three axes Lord IMU data.
% [imu_data_rotated] = rotate_imu_data (imu_message, rotation_angles, invert_rotation)
%
%
% Arguments-
%
% imu_message:     struct with message containing atleast three axes data, no time array
% rotation_angle:  rotation angles (deg) in the order [yaw, pitch, roll]
% invert_rotation: optional, numeric, if value is '1', then apply inverse of the rotation matrix
%                  useful for recovering raw values, before rotation was applied.
%
% Return Values-
%
% imu_data_rotated: same structure as imu_message but with rotations applied.

if nargin > 3 || nargin < 2
  disp ('Input error, check arguments.');
  return;
elseif nargin == 2
  invert_rotation = 0;
end

% find the number of data fields in imu_message
no_fields = numel (fieldnames(imu_message));

if no_fields < 3 || rem (no_fields, 3) ~= 0;
  disp ('Data input error, atleast three axes not found.');
  return;
end

% rotation angles
si    = rotation_angles(1); % Yaw
theta = rotation_angles(2); % Pitch
phi   = rotation_angles(3); % Roll

% rotation applied in the order Yaw --> Pitch --> Roll
T1 = [cosd(si)   -sind(si) 0           ; sind(si) cosd(si)  0         ; 0           0         1          ];
T2 = [cosd(theta) 0        -sind(theta); 0        1         0         ; sind(theta) 0         cosd(theta)];
T3 = [1           0        0           ; 0        cosd(phi) -sind(phi); 0           sind(phi) cosd(phi)  ];

% construct the rotation matrix
T_rotation = T3*T2*T1;

if invert_rotation == 1;
  % invert the matrix, usually to recover values before rotation was applied
  T_rotation = inv(T_rotation);
end

% apply the rotation matrix one data index at a time
ax = imu_message.AX;
ay = imu_message.AY;
az = imu_message.AZ;

gx = imu_message.GX;
gy = imu_message.GY;
gz = imu_message.GZ;

for ii = 1:length(ax)
  a_in = [ax(ii); ay(ii); az(ii)];
  g_in = [gx(ii); gy(ii); gz(ii)];

  a_out = T_rotation*a_in;
  g_out = T_rotation*g_in;

  imu_data_rotated.AX(ii) = a_out(1);
  imu_data_rotated.AY(ii) = a_out(2);
  imu_data_rotated.AZ(ii) = a_out(3);

  imu_data_rotated.GX(ii) = g_out(1);
  imu_data_rotated.GY(ii) = g_out(2);
  imu_data_rotated.GZ(ii) = g_out(3);

  clear a_in a_out g_in g_out;
end



