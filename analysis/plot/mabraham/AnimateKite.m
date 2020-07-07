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

function AnimateKite(c,rawm600,FPidx,in,varargin)
% Input arguments:
%
% c             ControllerA structure
% rawm600       A structure contained in m600geometry.mat
% FPidx         Array of indices of c.time to use for animation frames
% in            The complete logfile data structure from load_h5log.m
% varargin      Optional follow_kite boolean which sets axes limits

R = sqrt(c.crosswind.path_center_g.x.^2 + c.crosswind.path_center_g.y.^2);
path_azi = atan2(c.crosswind.path_center_g.y,c.crosswind.path_center_g.x);
path_ele = atan2(-c.crosswind.path_center_g.z,R);
angs = 0:360;
xcirc = cosd(angs);
ycirc = sind(angs);


%% Try to get the tether info. If it fails, don't plot the tether
try
    stime = in.Simulator.SimTelemetry.message.time;
    stime = stime - stime(1) + c.time(1);
    
    xposin = in.Simulator.SimTelemetry.message.tether.Xg_nodes.x';
    yposin = in.Simulator.SimTelemetry.message.tether.Xg_nodes.y';
    zposin = in.Simulator.SimTelemetry.message.tether.Xg_nodes.z';
    start_indin = double(in.Simulator.SimTelemetry.message.tether.start_ind);
    end_indin = double(in.Simulator.SimTelemetry.message.tether.end_ind);
    
    xpos = interp1(stime,xposin,c.time);
    ypos = interp1(stime,yposin,c.time);
    zpos = interp1(stime,zposin,c.time);
    start_ind = round(interp1(stime,start_indin,c.time));
    end_ind = round(interp1(stime,end_indin,c.time));
    tether_available = 1;
catch
    tether_available = 0;
end

%% Setup and an initial plotting of the flight path
plot3(c.state_est.Xg.y(FPidx),...
    c.state_est.Xg.x(FPidx),...
    -c.state_est.Xg.z(FPidx),'k-');
% Otherwise set standards
ax = axis;
[az,el] = view;

if mean(diff(c.time))>(1/15)
    FR = mean(diff(c.time));
else
    FR = 1/15; % seconds per frame
end

if tether_available
    try
        tethernodes = (start_ind(FPidx):end_ind(FPidx))+1;
        plot3(ypos(FPidx,tethernodes),...
            xpos(FPidx,tethernodes),...
            -zpos(FPidx,tethernodes),'.-')
    catch
        plot3([0; c.state_est.Xg.y(FPidx)],[0; c.state_est.Xg.x(FPidx)],[0; -c.state_est.Xg.z(FPidx)],'b-')
    end
end

% Check for inputs about axes, view and framerate
if isempty(varargin)
    followkiteflag = 0;
else
    followkiteflag = varargin{1};
end

%% Saving a video
savevideo = 1;
if savevideo
    vidObj = VideoWriter('Kite_Movie.avi');
    vidObj.FrameRate = 1/FR;
    open(vidObj);
end

%% Draw stuff

hold on
DrawM600(c.state_est.Xg.x(FPidx(1)),...
    c.state_est.Xg.y(FPidx(1)),...
    c.state_est.Xg.z(FPidx(1)),...
    c.state_est.dcm_g2b.d(:,:,FPidx(1))',...
    rawm600);

grid on

if tether_available
    try
        plot3(ypos(FPidx(1),:),xpos(FPidx(1),:),-zpos(FPidx(1),:),'.-')
    catch
    end
end

% This is how many indices of c.time to skip between frame captures
didx = max(1,floor(FR/mean(diff(c.time))));

% Loop through all the specified c.time indices, skipping didx of them to
% achieve the desired "real time" effect with the specified framerate
for jdx = 1:didx:numel(FPidx)
    
    % Clear the plot on each new frame
    cla
    
    % Plot the kite path
    plot3(c.state_est.Xg.y(FPidx(max(1,jdx-400):jdx)),...
        c.state_est.Xg.x(FPidx(max(1,jdx-400):jdx)),...
        -c.state_est.Xg.z(FPidx(max(1,jdx-400):jdx)),'k.');
    
    % Hold everything until we clear it again
    hold on
    
    try
        % The present kite position
        posnow = [c.state_est.Xg.x(FPidx(jdx));...
            c.state_est.Xg.y(FPidx(jdx));...
            c.state_est.Xg.z(FPidx(jdx))];
        
        % The tether nodes which are active now
        tethernodes = (start_ind(FPidx(jdx)):end_ind(FPidx(jdx)))+1;
        
        % Plot the tether nodes
        plot3(ypos(FPidx(jdx),tethernodes),...
            xpos(FPidx(jdx),tethernodes),...
            -zpos(FPidx(jdx),tethernodes),'.-')
        
        % Where the tether would intersect the kite
        bridlecenter = posnow + c.state_est.dcm_g2b.d(:,:,FPidx(jdx))*[0; in.parameters.system_params.wing.bridle_y_offset; 0];
        
        % The last tether node position
        tetherend = [xpos(FPidx(jdx),tethernodes(end));...
            ypos(FPidx(jdx),tethernodes(end));...
            zpos(FPidx(jdx),tethernodes(end))];
        
        % A vector pointing to the knot
        bridlevec =  tetherend - bridlecenter;
        bridleunit = bridlevec/norm(bridlevec);
        
        % The knot
        knot = posnow + bridleunit*in.parameters.system_params.wing.bridle_rad;
        
        % Calculate the hardpoints and plot the bridle
        hardpoint1 = [in.parameters.system_params.wing.bridle_pos.x(1);...
            in.parameters.system_params.wing.bridle_pos.y(1);...
            in.parameters.system_params.wing.bridle_pos.z(1)];
        
        hardpoint2 = [in.parameters.system_params.wing.bridle_pos.x(2);...
            in.parameters.system_params.wing.bridle_pos.y(2);...
            in.parameters.system_params.wing.bridle_pos.z(2)];
        
        hprot1 = posnow + c.state_est.dcm_g2b.d(:,:,FPidx(jdx))*hardpoint1;
        hprot2 = posnow + c.state_est.dcm_g2b.d(:,:,FPidx(jdx))*hardpoint2;
        plot3([hprot1(2) knot(2) hprot2(2)], [hprot1(1) knot(1) hprot2(1)], -[hprot1(3) knot(3) hprot2(3)],'b.-')
        plot3([knot(2) tetherend(2)],[knot(1) tetherend(1)],-[knot(3) tetherend(3)],'b.-')
        
    catch        
        plot3([0 c.state_est.Xg.y(FPidx(jdx))],[0 c.state_est.Xg.x(FPidx(jdx))],[0 -c.state_est.Xg.z(FPidx(jdx))],'b-')
    end
    
    % Plot the kite
    DrawM600(c.state_est.Xg.x(FPidx(jdx)),...
        c.state_est.Xg.y(FPidx(jdx)),...
        c.state_est.Xg.z(FPidx(jdx)),...
        c.state_est.dcm_g2b.d(:,:,FPidx(jdx))',...
        rawm600);
    
    % Calculations for a commanded crosswind circle plot
    DCM_g2cw = L2(path_ele(FPidx(jdx)))*L3(path_azi(FPidx(jdx)));
    circxyz = c.crosswind.path_radius_target(FPidx(jdx))*[0*xcirc; ycirc; xcirc];
    rotcirc = DCM_g2cw'*circxyz;
    
    % Plot the crosswind circle
    if (c.flight_mode(FPidx(jdx))==7)|(c.flight_mode(FPidx(jdx))==8)
        
        plot3(c.crosswind.path_center_g.y(FPidx(jdx))+rotcirc(2,:),...
            c.crosswind.path_center_g.x(FPidx(jdx))+rotcirc(1,:),...
            -c.crosswind.path_center_g.z(FPidx(jdx))-rotcirc(3,:));
        
        % Plot the wind aloft vector estimates
        if 0
            plot3(c.state_est.Xg.y(FPidx(jdx))+10*[0 c.state_est.wind_aloft_g.vector_f_slow.y(FPidx(jdx))],...
                c.state_est.Xg.x(FPidx(jdx))+10*[0 c.state_est.wind_aloft_g.vector_f_slow.x(FPidx(jdx))],...
                -c.state_est.Xg.z(FPidx(jdx))-10*[0 c.state_est.wind_aloft_g.vector_f_slow.z(FPidx(jdx))]);
            
            plot3(c.state_est.Xg.y(FPidx(jdx))+10*[0 c.state_est.wind_aloft_g.vector_f.y(FPidx(jdx))],...
                c.state_est.Xg.x(FPidx(jdx))+10*[0 c.state_est.wind_aloft_g.vector_f.x(FPidx(jdx))],...
                -c.state_est.Xg.z(FPidx(jdx))-10*[0 c.state_est.wind_aloft_g.vector_f.z(FPidx(jdx))]);
        end
    end
    
    % Grid, axis scaling, and title
    grid on
    axis equal
    title(['time = ' num2str(c.time(FPidx(jdx)),'%.2f')])
    
    % This will be used to set axes limits
    kitepos = [c.state_est.Xg.y(FPidx(jdx));...
        c.state_est.Xg.y(FPidx(jdx));...
        c.state_est.Xg.x(FPidx(jdx));...
        c.state_est.Xg.x(FPidx(jdx));...
        -c.state_est.Xg.z(FPidx(jdx));...
        -c.state_est.Xg.z(FPidx(jdx))]';
    
    % Set the axes limits depending on the user specification
    if followkiteflag
        axis(kitepos+20*[-1 1 -1 1 -1 1])
    else
        axis tight
    end    
    
    drawnow
    if savevideo
        currFrame = getframe(gcf);
        writeVideo(vidObj,currFrame);
    end
    
end

close(vidObj)
