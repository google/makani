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

function dcm = angle_to_dcm(r1, r2, r3, rotation_order)
%  ANGLE_TO_DCM    Find Direction Cosine Matrix (DCM) from rotation angles.
%   DCM = ANGLE_TO_DCM( R1, R2, R3, rotation_order ) calculates the direction
%
%   R1, R2, and R3 are scalars
%   rotation_order is a character array of rotations
%
%   Ex:
%   >> angle_to_dcm(3*pi, 5, pi/2, 'zxy')
%   >> angle_to_dcm(3*pi, 5, pi/2, 'zyz')
%
%   TODO: vectorize

ang = [r1, r2, r3];
dcm = eye(3);

for i = 1:length(rotation_order)
    switch rotation_order(i)
        case 'x'
            dcm = Rx(ang(i))*dcm;
        case 'y'
            dcm = Ry(ang(i))*dcm;
        case 'z'
            dcm = Rz(ang(i))*dcm;
    end
end


function Rx = Rx(r)
cr = cos(r);
sr = sin(r);
Rx = [ 1,   0,  0; ...
       0,  cr, sr; ...
       0, -sr, cr];

function Ry = Ry(r)
cr = cos(r);
sr = sin(r);
Ry = [ cr,  0,  -sr; ...
       0,   1,    0; ...
       sr,  0,   cr];

function Rz = Rz(r)
cr = cos(r);
sr = sin(r);
Rz = [cr, sr, 0;
     -sr, cr, 0; ...
      0,  0, 1];
