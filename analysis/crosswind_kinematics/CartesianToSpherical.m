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

function [R, Rd, Rdd,...
    psi, psid, psidd,...
    theta, thetad, thetadd] =...
    CartesianToSpherical(x,y,z,xd,yd,zd,xdd,ydd,zdd)

% Converts Cartesian Coordinates and their time derivatives into Spherical
% Coordinates and their time derivatives.

%% Radius and its time derivatives
R = sqrt(x^2 + y^2 + z^2);
Rd = 1/R*(x*xd + y*yd + z*zd);
Rdd = -Rd^2/R + (xd^2 + yd^2 + zd^2)/R + (x*xdd + y*ydd + z*zdd)/R;

%% Azimuth and Elevation
psi = atan2(y,x);
theta = asin(-z/R);

%% d/dt of the Azimuth and Elevation
% Chain Rule

dpsidx = -y/(x^2+y^2);
dpsidy = x/(x^2+y^2);

dthetadz = -1/(R*sqrt(1-z^2/R^2));
dthetadR = z/(R^2*sqrt(1-z^2/R^2));

psid = dpsidx*xd + dpsidy*yd;
thetad = dthetadz*zd + dthetadR*Rd;

%% d^2/dt^2 of the Azimuth and Elevation
% Also Chain Rule
d_dpsidx_dx = 2*x*y/(x^2+y^2)^2;
d_dpsidx_dy = (y^2-x^2)/(x^2+y^2)^2;
dpsidxd = d_dpsidx_dx*xd + d_dpsidx_dy*yd;

d_dpsidy_dx = d_dpsidx_dy; % Chain rule identity
d_dpsidy_dy = -2*x*y/(x^2+y^2)^2;
dpsidyd = d_dpsidy_dx*xd + d_dpsidy_dy*yd;

d_dthetadz_dz = -z/(R^3*(1-z^2/R^2)^(3/2));
d_dthetadz_dR = R^2*sqrt(1-z^2/R^2)/(R^2+z^2)^2;
dthetadzd = d_dthetadz_dz*zd + d_dthetadz_dR*Rd;

d_dthetadR_dz = d_dthetadz_dR; % Chain rule identity
d_dthetadR_dR = (z^3-2*R^2*z)/(R^3*(R^2-z^2)*sqrt(1-z^2/R^2));
dthetadRd = d_dthetadR_dz*zd + d_dthetadR_dR*Rd;

psidd = dpsidxd*xd + dpsidx*xdd + dpsidyd*yd + dpsidy*ydd;
thetadd = dthetadzd*zd + dthetadz*zdd + dthetadRd*Rd + dthetadR*Rdd;
