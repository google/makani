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

% Use this to test SphereKinematics.m for a single set of inputs.

Rs = 430;

psi_s = 0;
theta_s = pi/4;

V = 50;
phi_s = 0;

psid_s = V*cos(phi_s)/Rs/cos(theta_s);
thetad_s = -V*sin(phi_s)/Rs;

psidd_s = 0;
thetadd_s = 0;

alpha = 0;
beta = 0;
phi_a = 0;

alphad = 0;
betad = 0;
phid_a = 0;

v_ag_g = [10; 0; 0];

[r_ok_g, DCM_g2k, v_kg_g ,omega_kg_k, v_ka_g] =...
    SphereKinematics(Rs, psi_s, theta_s,...
    psid_s, thetad_s,...
    psidd_s, thetadd_s,...
    alpha, beta, phi_a,...
    alphad, betad, phid_a, v_ag_g);