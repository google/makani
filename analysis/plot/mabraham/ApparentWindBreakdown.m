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

% This script does the following:
% - Takes a time derivative of our estimated alpha and beta
% - Computes the contributions to d/dt(alpha) and d/dt(beta) from the
%       kite's inertial motion (accelerations & rotations)
% - Makes plots showing which parts of d/dt(alpha) and d/dt(beta) can be
%       explained by kite motion. The rest is labeled "gust".
%
% See the appendix of ECR 337 for mathematical details:
% https://docs.google.com/document/d/1XdXnc6q47uysEIj__LJym9_o_GMgd8TrWMClQtAoU4o/edit#heading=h.pt1a3cp50c9s


%% Velocity of the atmosphere w.r.t. the kite, components in frame b
ua = -c.state_est.apparent_wind.vector.x;
va = -c.state_est.apparent_wind.vector.y;
wa = -c.state_est.apparent_wind.vector.z;

%% The magnitude of the freestream velocity as seen by the kite
Vinf = sqrt(ua.^2 + va.^2 + wa.^2);

%% Velocity of the kite w.r.t. the ground, components in frame b
u = c.state_est.Vb.x;
v = c.state_est.Vb.y;
w = c.state_est.Vb.z;

%% Angular velocity of the kite w.r.t. the ground, components in frame b
p = c.state_est.pqr_f.x;
q = c.state_est.pqr_f.y;
r = c.state_est.pqr_f.z;

%% Collect into 3xn arrays
pqr = [p'; q'; r'];
uvw = [u'; v'; w'];
uvwa = [ua'; va'; wa'];

%% Useful denominators
alphadenom = ua.^2 + wa.^2;
betadenom = sqrt(1-(va./Vinf).^2);

%% The aerodynamic angles from telemetry
alpha = c.state_est.apparent_wind.sph.alpha;
beta = c.state_est.apparent_wind.sph.beta;

%% A lowpass filter to get rid of noise
wn = 5*2*pi; % Cutoff frequency in rad/sec
zeta = 1;    % Damping ratio
filtsys = tf(wn^2,[1 2*zeta*wn wn^2]); % 2nd order lowpass filter
time = linspace(c.time(1),c.time(end),numel(c.time));
time = time - time(1);

%% Take time derivatives and apply lowpass filtering

% Aero angles
alphadot_measured = ForwardBackwardFilter(time,alpha,filtsys,1);
betadot_measured = ForwardBackwardFilter(time,beta,filtsys,1);

% Inertial velocities in frame b
udot = ForwardBackwardFilter(time,u,filtsys,1);
vdot = ForwardBackwardFilter(time,v,filtsys,1);
wdot = ForwardBackwardFilter(time,w,filtsys,1);

% Aero velocities in frame b
uadot = ForwardBackwardFilter(time,ua,filtsys,1);
vadot = ForwardBackwardFilter(time,va,filtsys,1);
wadot = ForwardBackwardFilter(time,wa,filtsys,1);

% Freestream velocity
Vinfdot = ForwardBackwardFilter(time,Vinf,filtsys,1);

if 0 % Filtering these signals may not be necessary
    p = ForwardBackwardFilter(time,p,filtsys,0);
    q = ForwardBackwardFilter(time,q,filtsys,0);
    r = ForwardBackwardFilter(time,r,filtsys,0);
end

%% Collect into 3xn arrays
pqr = [p'; q'; r'];
uvw = [u'; v'; w'];
uvwa = [ua'; va'; wa'];
uvwdot = [udot'; vdot'; wdot'];
uvwadot = [uadot'; vadot'; wadot'];

%% Cross prodcuts
pqrxuvw = zeros(3,numel(c.time));
pwrxuvwa = zeros(3,numel(c.time));

for idx = 1:numel(c.time)
    pqrxuvw(:,idx) = cross(pqr(:,idx),uvw(:,idx));
    pqrxuvwa(:,idx) = cross(pqr(:,idx),uvwa(:,idx));
end

%% Components of uvwadot

% The kite accelerates = F / m
Foverm = uvwdot + pqrxuvw;

% The kite rotates with respect to the wind
% Here the wind is constructed from inertial velocities (uvw) and apparent
% wind velocities (uvwa) and the cross product is distributed.
pqrxuvww = -pqrxuvwa + pqrxuvw;

% Everything we can't account for otherwise
gusts = Foverm - pqrxuvwa - uvwadot;


%% Alphadot breakdown
% Total alpha dot reconstructed from inertial sensors
alphadot_reconstructed = ua.*wadot - wa.*uadot;

% Individual terms
% Kite accelerates
alphadot_fm = Foverm(3,:).*ua' + Foverm(1,:).*(-wa');

% Kite rotates
alphadot_uvw = -pqrxuvw(3,:).*ua'-pqrxuvw(1,:).*(-wa');
alphadot_uvww = pqrxuvww(3,:).*ua'+ pqrxuvww(1,:).*(-wa');

% Everything else
alphadot_gusts = -gusts(3,:).*ua'-gusts(1,:).*(-wa');

%% Betadot breakdown
% Total beta dot reconstructed from inertial sensors
betadot_reconstructed = vadot./Vinf - va.*Vinfdot;

% d/dt(Vinf) due to acceleration
Vinfdot_Foverm = (ua.*uadot+va.*vadot+wa.*wadot)./Vinf;
% d/dt(Vinf) due to gusts
Vinfdot_gusts = Vinfdot - Vinfdot_Foverm;

% Kite accelerates
betadot_fm = Foverm(2,:)./Vinf' + (-va.*Vinfdot_Foverm./Vinf.^2)';

% Kite rotates
betadot_uvw = -pqrxuvw(2,:)./Vinf';
betadot_uvww = pqrxuvww(2,:)./Vinf';

% Everything else
betadot_gusts = -gusts(2,:)./Vinf' + (-va.*Vinfdot_gusts./Vinf.^2)';

%% Plot

figure('name','Alphadot Breakdown')
plot(c.time,alphadot_measured'*180/pi)
title('Alphadot Breakdown')
hold on
plot(c.time,(alphadot_fm')./alphadenom*180/pi)
plot(c.time,(alphadot_uvw'+alphadot_uvww')./alphadenom*180/pi)
plot(c.time,(alphadot_gusts')./alphadenom*180/pi)
%plot(c.time,(alphadot)./alphadenom*180/pi)
legend('alpha dot total','Kite Accelerates','Kite Rotates','Gusts')
grid on

figure('name','Betadot Breakdown')
plot(c.time,betadot_measured'*180/pi)
title('Betadot Breakdown')
hold on
plot(c.time,(betadot_fm')./betadenom*180/pi)
plot(c.time,(betadot_uvw'+betadot_uvww')./betadenom*180/pi)
plot(c.time,(betadot_gusts')./betadenom*180/pi)
legend('beta dot total','Kite Accelerates','Kite Rotates','Gusts')
grid on
ylabel('[deg/sec]')

