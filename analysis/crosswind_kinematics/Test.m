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

% This script is used to test SphereKinematics.m
%% Saving a video
savevideo = 1;
if savevideo
    vidObj = VideoWriter('Kite_Movie.avi');
    vidObj.FrameRate = 15;
    open(vidObj);
end

%% Wind vector
v_ag_g = [10; 0; 0];

%% Sphere Radius
R = 430;

%% Time
t0 = 0;
tfinal = 20;
dt = .05;
t = t0:dt:tfinal;

%% Random Time History of Sphere Angles
[psi,psid,psidd] = SineWaveAndDerivatives(rand*pi/2,rand/2,rand,t);
[theta,thetad,thetadd] = SineWaveAndDerivatives(rand*pi/2,rand,rand,t);

%% Other Kite Motion Commands
[alpha,alphad,alphadd] = SineWaveAndDerivatives(1*rand*pi/8,6*rand,rand,t);
[beta,betad,betadd] = SineWaveAndDerivatives(1*rand*pi/8,5*rand,rand,t);
[phi_a,phi_ad,phi_add] = SineWaveAndDerivatives(1*rand*pi/8,8*rand,rand,t);

%% Loop initialization
r_ok_g = zeros(3,numel(t));
DCM_g2b = zeros(3,3,numel(t));
v_kg_g = zeros(3,numel(t));
a_kg_g = zeros(3,numel(t));
omega_bg_b = zeros(3,numel(t));
pqrest = zeros(3,numel(t));

%% Compute kinematics based on Spherical Coordinates
for idx = 1:numel(t)
    cla
    [r_ok_g(:,idx), DCM_g2b(:,:,idx), v_kg_g(:,idx) ,omega_bg_b(:,idx), a_kg_g(:,idx)] =...
        SphereKinematics(R, psi(idx), theta(idx),...
        psid(idx), thetad(idx),...
        psidd(idx), thetadd(idx),...
        alpha(idx), beta(idx), phi_a(idx),...
        alphad(idx), betad(idx), phi_ad(idx), v_ag_g);
    view(-80,10)
    drawnow

    if savevideo
        currFrame = getframe(gcf);
        writeVideo(vidObj,currFrame);
    end

    % Calculate pqr estimate from d/dt(DCM_g2b)
    % This uses finite differencing so that it can be compared to the
    % analytical result.
    ddtdcm = (DCM_g2b(:,:,idx) - DCM_g2b(:,:,max(idx-1,1)))/dt;
    omskew = -DCM_g2b(:,:,idx)'*ddtdcm;
    pqrest(:,idx) = DCM_g2b(:,:,idx)*[omskew(3,2); -omskew(3,1); omskew(2,1)];

    % Print a comparison between the two pqr estimates
    %[omega_bg_b(:,idx), pqrest(:,idx), pqrest(:,idx)./omega_bg_b(:,idx)]
end

close(vidObj)

%% Convert Cartesian Coordinates to Spherical Coordinates
x = r_ok_g(1,:);
y = r_ok_g(2,:);
z = r_ok_g(3,:);

xd = v_kg_g(1,:);
yd = v_kg_g(2,:);
zd = v_kg_g(3,:);

xdd = a_kg_g(1,:);
ydd = a_kg_g(2,:);
zdd = a_kg_g(3,:);

for idx = 1:numel(t)
    [R2(idx), Rd2(idx), Rdd2(idx),...
        psi2(idx), psid2(idx), psidd2(idx),...
        theta2(idx), thetad2(idx), thetadd2(idx)] =...
        CartesianToSpherical(x(idx),y(idx),z(idx),...
        xd(idx),yd(idx),zd(idx),...
        xdd(idx),ydd(idx),zdd(idx));
end

%% Plots to confirm accuracy of algorithms
figure
hold on
title('Checking CartesianToSpherical.m')
plot(t,psi*180/pi)
grid on
plot(t,theta*180/pi)
plot(t,psid*180/pi)
plot(t,thetad*180/pi)
plot(t,psidd*180/pi)
plot(t,thetadd*180/pi)
plot(t,psi2*180/pi,'.')
plot(t,theta2*180/pi,'.')
plot(t,psid2*180/pi,'.')
plot(t,thetad2*180/pi,'.')
plot(t,psidd2*180/pi,'.')
plot(t,thetadd2*180/pi,'.')
ylabel('[deg/sec]')
xlabel('time [sec]')

figure
plot(t,omega_bg_b*180/pi)
hold on
plot(t,pqrest*180/pi,'.')
title('pqr estimates')
legend('From SphereKinematics.m','Reconstructed from d/dt(DCM\_g2b)')
grid on
ylabel('deg/sec')
