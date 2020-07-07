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

%% Setup
IncludeBaselinePlots = 1;
IncludeHoverPlots = 0;
IncludeCrosswindPlots = 1;
IncludeGPSPlots = 0;

% The list of all figures with time on the x axis.
% We append this list with each new figure.
timehistoryfigurelist = [];

if IncludeBaselinePlots
    
    %% Flight Mode
    h1 = figure('name','flt mode');
    plot(c.time,c.flight_mode)
    grid
    title('Flight Mode')
    xlabel('time [sec]')
    ylabel('flt mode')
    str = {'PilotHover = 0',...
        'Perched = 1',...
        'HoverAscend = 2',...
        'HoverPayOut = 3',...
        'HoverFullLength = 4',...
        'HoverAccel = 5',...
        'TransIn = 6',...
        'CrosswindNormal = 7',...
        'CrosswindPrepTransOut = 8',...
        'HoverTransOut = 9',...
        'HoverReelIn = 10',...
        'HoverDescend = 11',...
        'OffTether = 12',...
        'HoverTransformGsUp = 13',...
        'HoverTransformGsDown = 14',...
        'HoverPrepTransformGsUp = 15',...
        'HoverPrepTransformGsDown = 16'};
    h1 = annotation('textbox',[.8 .6 .1 .1],'String',str,'BackgroundColor','white','FontSize',8);
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Power Generated
    try
        figure('name','power')
        plot(c.time,c.power/1000)
        grid on
        xlabel('time [sec]')
        ylabel('kW Generated')
        title('Total Bus Power')
        legend('Bus Power')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
    catch
    end
    
    %% Joystick Sticks
    figure('name', 'Joystick Sticks')
    plot(c.time,c.control_input.joystick.throttle)
    hold on
    plot(c.time,c.control_input.joystick.roll)
    plot(c.time,c.control_input.joystick.pitch)
    plot(c.time,c.control_input.joystick.yaw)
    legend('Throttle Stick','Roll Stick','Pitch Stick','Yaw Stick')
    grid on
    xlabel('time [sec]')
    ylabel('NonDim')
    title('Joystick Sticks')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Payout
    figure('name', 'Payout')
    plot(c.time,c.state_est.winch.payout)
    legend('state\_est.payout')
    grid on
    xlabel('time [sec]')
    ylabel('m')
    title('Payout')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Tether Elevation
    try
        figure('name', 'Tether Elevation')
        plot(c.time,c.state_est.tether_ground_angles.elevation*180/pi);
        legend('state\_est.tether\_elevation.position')
        xlabel('time [sec]')
        ylabel('[DEG]')
        grid on
        title('Tether Elevation')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
    catch
    end
    
    
    %% Rotor Speed CMDs
    if ~isfield(c.control_output,'rotors')
        c.control_output.rotors = (c.control_output.motor_speed_lower_limit+c.control_output.motor_speed_upper_limit)/2;
    end
    figure('name','Rotor Speed CMDs')
    plot(c.time,c.control_output.rotors)
    grid on
    xlabel('time [sec]')
    ylabel('rad/sec')
    title('Rotor Speed CMD')
    %legend('sbo','sbi','pbi','pbo','pto','pti','sti','sto')
    legend('sbo\_cmd','sbi\_cmd','pbi\_cmd','pbo\_cmd','pto\_cmd','pti\_cmd','sti\_cmd','sto\_cmd')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Rotor Speeds
    figure('name','Rotor Speeds')
    plot(c.time,c.control_input.rotors)
    grid on
    xlabel('time [sec]')
    ylabel('rad/sec')
    title('Rotor Speeds')
    legend('sbo','sbi','pbi','pbo','pto','pti','sti','sto')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Wing Control Surfaces
    figure('name','Wing Control Surfaces')
    plot(c.time,c.control_output.flaps(1:6,:)'*180/pi)
    hold on
    grid on
    xlabel('time [sec]')
    ylabel('[DEG]')
    legend('1','2','4','5','7','8')
    title('Wing Control Surfaces')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Elevator and Rudder
    figure('name','elv & rud cmd & meas')
    plot(c.time,c.control_output.flaps(7,:)*180/pi)
    hold on
    plot(c.time,c.control_input.flaps(7,:)*180/pi)
    plot(c.time,c.control_output.flaps(8,:)*180/pi)
    plot(c.time,c.control_input.flaps(8,:)*180/pi)
    grid on
    title('Controller Elevator and Rudder Cmd & Meas')
    xlabel('time [sec]')
    ylabel('[DEG]')
    legend('cmdE','measE','cmdR','measR')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Thrust
    figure('name','Thrust')
    plot(c.time,c.thrust_moment.thrust,'b-');
    hold on
    plot(c.time,c.thrust_moment_avail.thrust,'r--')
    grid on
    legend('Thrust','Thrust Available')
    title('Thrust')
    xlabel('time [sec]')
    ylabel('Newtons')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Rotor Moments
    figure('name','Rotor Moments')
    plot(c.time,c.thrust_moment.moment.x,'r-');
    hold on
    plot(c.time,c.thrust_moment.moment.y,'b-');
    plot(c.time,c.thrust_moment.moment.z,'g-');
    plot(c.time,c.thrust_moment_avail.moment.x,'m--');
    plot(c.time,c.thrust_moment_avail.moment.y,'c--');
    plot(c.time,c.thrust_moment_avail.moment.z,'k--');
    grid on
    legend('Roll Moment','Pitch Moment','Yaw Moment','Roll Moment Avail','Pitch Moment Avail','Yaw Moment Avail')
    title('Rotor Moments')
    xlabel('time [sec]')
    ylabel('N*m')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Moment Deltas
    figure('name','Deltas')
    plot(c.time,c.thrust_moment_avail.thrust-c.thrust_moment.thrust,'k')
    hold on
    plot(c.time,c.thrust_moment_avail.moment.x-c.thrust_moment.moment.x,'r-');
    plot(c.time,c.thrust_moment_avail.moment.y-c.thrust_moment.moment.y,'g-');
    plot(c.time,c.thrust_moment_avail.moment.z-c.thrust_moment.moment.z,'b-');
    grid on
    legend('Thrust Delta','Roll Delta','Pitch Delta','Yaw Delta')
    title('Deltas')
    xlabel('time [sec]')
    ylabel('N or N*m')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Alt Est
    figure('name','Altitude')
    plot(c.time,-c.state_est.Xg.z)
    hold on
    %plot(c.time,-c.crosswind.path_center_g.z,'--')
    plot(c.time,-c.hover.wing_pos_g_cmd.z,'--')
    grid on
    title('Altitude')
    xlabel('time [sec]')
    ylabel('alt (m)')
    legend('est','cmd')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Airspeed
    figure('name','Airspeed')
    plot(c.time,c.state_est.apparent_wind.sph_f.v,'b')
    hold on
    plot(c.time,c.crosswind.airspeed_cmd,'r--')
    grid on
    xlabel('time [sec]')
    ylabel('[m/s]')
    legend('Airspeed EST','Airspeed CMD')
    title('Airspeed')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Kite Angular Rates
    figure('name','Kite Angular Rates')
    plot(c.time,[c.state_est.pqr_f.x,c.state_est.pqr_f.y,c.state_est.pqr_f.z]*180/pi)
    grid on
    title('Kite Angular Rates')
    xlabel('time [sec]')
    ylabel('rate (deg/sec)')
    hold on
    plot(c.time,c.crosswind.pqr_cmd.x*180/pi,'--')
    plot(c.time,c.crosswind.pqr_cmd.y*180/pi,'--')
    plot(c.time,c.crosswind.pqr_cmd.z*180/pi,'--')
    legend('p','q','r','pcmd','qcmd','rcmd')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Vg
    figure('name','Vg')
    plot(c.time,c.state_est.Vg_f.x)
    hold on
    plot(c.time,c.state_est.Vg_f.y)
    plot(c.time,c.state_est.Vg_f.z)
    title('Vg')
    xlabel('time [sec]')
    ylabel('[m/s]')
    legend('x','y','z')
    grid on
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Vb
    try
        figure('name','Vb')
        plot(c.time,c.state_est.Vb_f.x)
        hold on
        plot(c.time,c.state_est.Vb_f.y)
        plot(c.time,c.state_est.Vb_f.z)
        title('Vb')
        xlabel('time [sec]')
        ylabel('[m/s]')
        legend('x','y','z')
        grid on
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
    catch
    end
    
    %% Ab
    figure('name','Ab')
    plot(c.time,c.state_est.Ab_f.x)
    hold on
    plot(c.time,c.state_est.Ab_f.y)
    plot(c.time,c.state_est.Ab_f.z)
    title('Ab')
    xlabel('time [sec]')
    ylabel('m/s^2')
    legend('x','y','z')
    grid on
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Xg
    figure('name','Xg')
    plot(c.time,c.state_est.Xg.x)
    hold on
    plot(c.time,c.state_est.Xg.y)
    plot(c.time,c.state_est.Xg.z)
    title('Xg')
    xlabel('time [sec]')
    ylabel('m')
    legend('x','y','z')
    grid on
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% GPS Differences
    figure('name','GPS Differences')
    gpsaxlist = [];
    subplot(3,1,1)
    plot(c.time,c.estimator.gps.Xg.x(3,:)-c.estimator.gps.Xg.x(4,:))
    xlabel('time [sec]')
    ylabel('[m]')
    title('Xg.x Differences 3 - 4')
    grid on
    gpsaxlist = [gpsaxlist gca];
    subplot(3,1,2)
    plot(c.time,c.estimator.gps.Xg.y(3,:)-c.estimator.gps.Xg.y(4,:))
    xlabel('time [sec]')
    ylabel('[m]')
    title('Xg.y Differences 3 - 4')
    grid on
    gpsaxlist = [gpsaxlist gca];
    subplot(3,1,3)
    plot(c.time,c.estimator.gps.Xg.z(3,:)-c.estimator.gps.Xg.z(4,:))
    xlabel('time [sec]')
    ylabel('[m]')
    title('Xg.z Differences 3 - 4')
    grid on
    gpsaxlist = [gpsaxlist gca];
    linkaxes(gpsaxlist,'x')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    
    %% Aero Angles
    figure('name','Aero Angles')
    plot(c.time,180/pi*c.state_est.apparent_wind.sph_f.alpha,'b-')
    hold on
    plot(c.time,180/pi*c.crosswind.alpha_cmd,'c--')
    plot(c.time,180/pi*c.state_est.apparent_wind.sph_f.beta,'r-')
    plot(c.time,180/pi*c.crosswind.beta_cmd,'m--')
    grid on
    legend('AoA Est','AoA CMD','SS Est','SS CMD')
    xlabel('time [sec]')
    ylabel('[DEG]')
    title('Aerodynamic Angles')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Inertial Aero Angles
    % Angles computed in the style of alpha and beta but using the inertial
    % velocity vector instead of the airspeed velocity vector. These are angles
    % away from the path tangent.
    
    u = c.state_est.Vb_f.x;
    v = c.state_est.Vb_f.y;
    w = c.state_est.Vb_f.z;
    V = sqrt(u.^2 + v.^2 + w.^2);
    beta_inertial = asin(v./V);
    alpha_inertial = atan2(w,u);
    figure('name','Inertial Aero Angles')
    hold on
    plot(c.time,180/pi*alpha_inertial,'b-')
    plot(c.time,180/pi*beta_inertial,'r-')
    legend('Inertial Alpha','Inertial Beta')
    xlabel('time [sec]')
    ylabel('[DEG]')
    grid on
    title('Angles Away from the Flight Path')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Apparent Wind Vector Stuff
    for idx = 1:numel(c.time)
        appwind_bnow = [c.state_est.apparent_wind.vector.x(idx);...
            c.state_est.apparent_wind.vector.y(idx);...
            c.state_est.apparent_wind.vector.z(idx)];
        appwind_g(:,idx) = c.state_est.dcm_g2b.d(:,:,idx)*appwind_bnow;
        wind_g(:,idx) = c.state_est.dcm_g2b.d(:,:,idx)*(appwind_bnow + [u(idx); v(idx); w(idx)]);
        appwind_b2(:,idx) = c.state_est.dcm_g2b.d(:,:,idx)'*...
            [c.state_est.wind_g.vector.x(idx);...
            c.state_est.wind_g.vector.y(idx);...
            c.state_est.wind_g.vector.z(idx)] - [u(idx); v(idx); w(idx)];
    end
    try
        figure('name','Apparent Wind')
        hold on
        plot(c.time,c.state_est.apparent_wind.vector.x)
        plot(c.time,c.state_est.apparent_wind.vector.y)
        plot(c.time,c.state_est.apparent_wind.vector.z)
        plot(c.time,appwind_b2,'--')
        legend('State Est App Wind x',...
            'State Est App Wind y',...
            'State Est App Wind z',...
            'Back Calc App Wind x',...
            'Back Calc App Wind y',...
            'Back Calc App Wind z')
        xlabel('time [sec]')
        ylabel('[m/s]')
        grid on
        title('Apparent Wind')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
        alpha2 = atan2(-appwind_b2(3,:),-appwind_b2(1,:));
        V2 = sqrt(appwind_b2(1,:).^2+appwind_b2(2,:).^2+appwind_b2(3,:).^2);
        beta2 = asin(-appwind_b2(2,:)./V2);
    catch
    end
    
    %% Aero Angles 2
    if 0
        figure('name','Aero Angles 2')
        plot(c.time,180/pi*c.state_est.apparent_wind.sph.alpha,'b-')
        hold on
        plot(c.time,180/pi*c.state_est.apparent_wind.sph.beta,'r-')
        plot(c.time,180/pi*alpha2,'c-')
        plot(c.time,180/pi*beta2,'m-')
        
        grid on
        legend('alpha Est','beta Est','alpha Back Calc','beta Back Calc')
        xlabel('time [sec]')
        ylabel('[DEG]')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
    end
    
    %% Wind Components
    if 0
        try
            figure('name','Wind')
            hold on
            plot(c.time,c.state_est.wind_g.vector.x)
            plot(c.time,c.state_est.wind_g.vector.y)
            plot(c.time,c.state_est.wind_g.vector.z)
            plot(c.time,wind_g(1,:),'--')
            plot(c.time,wind_g(2,:),'--')
            plot(c.time,wind_g(3,:),'--')
            legend('State Est Wind g.x','State Est Wind g.y',...
                'State Est Wind g.z',...
                'Back Calc Wind g.x',...
                'Back Calc Wind g.y',...
                'Back Calc Wind g.z')
            grid on
            ylabel('[m/s]')
            xlabel('time [sec]')
            timehistoryfigurelist = [timehistoryfigurelist; gcf];
        catch
        end
    end
    
    %% Wind Speed
    figure('name','Wind Speed')
    hold on
    plot(c.time,c.state_est.wind_g.speed_f)
    legend('wind\_g.speed\_f')
    if isfield(c.state_est,'wind_aloft_g')
        plot(c.time,c.state_est.wind_aloft_g.speed_f)
        plot(c.time,c.state_est.wind_aloft_g.speed_f_playbook)
        legend('wind\_g.speed\_f','wind\_aloft\_g.speed\_f','wind\_aloft\_g.speed\_f\_playbook')
    end
    grid on
    
    xlabel('time [sec]')
    ylabel('[m/s]')
    title('Wind Speed Estimates')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Wind Dir
    figure('name','Wind Direction')
    hold on
    plot(c.time,c.state_est.wind_g.dir_f*180/pi)
    grid on
    xlabel('time [sec]')
    ylabel('[DEG]')
    legend('wind\_g.dir\_f')
    title('Wind Direction Estimates')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    
    %% Tether Angles
    figure('name','tether angles')
    plot(c.time,c.state_est.tether_force_b.sph.roll*180.0/pi,'r',c.time,c.crosswind.tether_roll_cmd*180.0/pi,'r--')
    hold on
    plot(c.time,c.state_est.tether_force_b.sph.pitch*180.0/pi,'b')
    grid
    title('Tether Angles')
    xlabel('time [sec]')
    ylabel('angle [deg]')
    legend('Tether Roll','Tether Roll cmd','Tether Pitch')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Tether Tension
    figure('name','tether tension')
    plot(c.time,[c.state_est.tether_force_b.tension_f,...
        sqrt(c.state_est.tether_force_b.vector_f.x.^2 + ...
        c.state_est.tether_force_b.vector_f.y.^2)],'-');
    grid
    hold on
    set(gca,'ColorOrderIndex',1)    % reset line color palette to original
    plot(c.time,[c.hover.tension_cmd,c.hover.horizontal_tension_cmd],'--')
    hold off
    title('Tether Tension')
    xlabel('time [sec]')
    ylabel('tension [N]')
    legend('meas total','horiz est')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Tether force vector
    figure('name','tether force vector')
    plot(c.time,c.state_est.tether_force_b.vector_f.x)
    hold on
    plot(c.time,c.state_est.tether_force_b.vector_f.y)
    plot(c.time,c.state_est.tether_force_b.vector_f.z)
    legend('x','y','z')
    xlabel('time [sec]')
    ylabel('N')
    grid on
    title('Tether Force Vector (frame b)')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Tether Radius
    if ~isfield(c,'TetherRadius')
        c.TetherRadius = sqrt(c.state_est.Xg.x.^2+c.state_est.Xg.y.^2+c.state_est.Xg.z.^2);
    end
    figure('name','Tether Radius')
    plot(c.time,c.TetherRadius)
    grid on
    hold on
    xlabel('time [sec]')
    ylabel('m')
    legend('Norm of Xg')
    title('Tether Radius')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
end

%% Crosswind Plots
if IncludeCrosswindPlots
    
    %% Complementary Filter Sideslip
    if isfield(c.estimator,'apparent_wind_cf')
        figure('name','CF Sideslip')
        plot(c.time,c.estimator.apparent_wind_pitot.beta*180/pi)
        hold on
        plot(c.time,c.estimator.apparent_wind_cf.beta*180/pi)
        grid on
        xlabel('time [sec]')
        ylabel('[DEG]')
        legend('\beta\_pitot','\beta\_cf')
        title('Sideslip Estimate')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
        
        %% Complementary Filter Angle of Attack
        figure('name','CF Angle of Attack')
        plot(c.time,c.estimator.apparent_wind_pitot.alpha*180/pi)
        hold on
        plot(c.time,c.estimator.apparent_wind_cf.alpha*180/pi)
        grid on
        xlabel('time [sec]')
        ylabel('[DEG]')
        legend('\alpha\_pitot','\alpha\_cf')
        title('Angle of Attack Estimate')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
        
        %% Complementary Filter Airspeed
        figure('name','CF Airspeed')
        plot(c.time,c.estimator.apparent_wind_pitot.v)
        hold on
        plot(c.time,c.estimator.apparent_wind_cf.v)
        grid on
        xlabel('time [sec]')
        ylabel('[m/s]')
        legend('v\_pitot','v\_cf')
        title('Airspeed Estimate')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
    end
    
    %% Curvatures
    figure('name','Curvatures')
    plot(c.time,c.crosswind.k_aero_cmd,'--')
    hold on
    plot(c.time,c.crosswind.k_aero_curr)
    plot(c.time,c.crosswind.k_geom_cmd,'--')
    plot(c.time,c.crosswind.k_geom_curr)
    grid on
    xlabel('time [sec]')
    ylabel('k')
    legend('k aero cmd','k aero curr','k geom cmd','k geom curr')
    title('Curvatures')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Crosswind Position
    figure('name','Crosswind Position')
    plot(c.time,c.crosswind.current_pos_cw.x)
    hold on
    plot(c.time,c.crosswind.current_pos_cw.y)
    plot(c.time,c.crosswind.target_pos_cw.x,'--')
    plot(c.time,c.crosswind.target_pos_cw.y,'--')
    grid on
    xlabel('time [sec]')
    ylabel('m')
    legend('x','y','x target','y target')
    title('Crosswind Position')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Sideslip Command
    figure('name','Sideslip Command')
    plot(c.time,c.crosswind.beta_cmd*180/pi,'o-')
    grid on
    xlabel('time [sec]')
    ylabel('[DEG]')
    legend('beta\_cmd')
    title('Sideslip Command')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Loop Radius
    try
        figure('name','Loop Radius')
        LoopRadiusCMD = c.crosswind.path_radius_target;
        LoopRadius = sqrt( (c.crosswind.current_pos_cw.x).^2 + (c.crosswind.current_pos_cw.y).^2  );
        plot(c.time,LoopRadius)
        hold on
        plot(c.time,LoopRadiusCMD,'--')
        grid on
        xlabel('time [sec]')
        ylabel('m')
        legend('Loop Radius','Loop Radius Command')
        title('Loop Radius')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
    catch
    end
    
    %% Cosstrack Integrator
    if isfield(c.crosswind,'int_crosstrack')
        figure('name','Crosstrack Integrator')
        plot(c.time,c.crosswind.int_crosstrack)
        grid on
        xlabel('time [sec]')
        ylabel('1/m')
        legend('Crosstrack Integrator')
        title('Crosstrack Integrator')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
    end
    
    if isfield(c.crosswind,'beta_harmonic_state')
        figure('name','Beta Harmonic State')
        plot(c.time,c.crosswind.beta_harmonic_state)
        grid on
        xlabel('time [sec]')
        legend('sine state','cosine state')
        title('Beta Harmonic State')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
    end
    
    %% Path Center
    figure('name','CrossWind Path Center')
    plot(c.time,c.crosswind.path_center_g.x)
    hold on
    plot(c.time,c.crosswind.path_center_g.y)
    plot(c.time,c.crosswind.path_center_g.z)
    legend('x','y','z')
    grid on
    ylabel('[m]')
    xlabel('time [sec]')
    title('Crosswind Path Center, Frame g')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Loop Angle
    figure('name','Loop Angle')
    plot(c.time,c.crosswind.loop_angle*180/pi)
    grid on
    hold on
    xlabel('time [sec]')
    ylabel('[DEG]')
    legend('Loop Angle')
    title('Loop Angle')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
end

if IncludeHoverPlots
    
    %% Hover Pos Error
    
    figure('name','Hover Pos b Error')
    plot(c.time, c.hover.wing_pos_b_error.x)
    hold on
    plot(c.time, c.hover.wing_pos_b_error.y)
    plot(c.time, c.hover.wing_pos_b_error.z)
    grid on
    xlabel('time [sec]')
    ylabel('m')
    legend('x','y','z')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Hover Vel Error
    
    figure('name','Hover Vel b Error')
    plot(c.time, c.hover.wing_vel_b_error.x)
    hold on
    plot(c.time, c.hover.wing_vel_b_error.y)
    plot(c.time, c.hover.wing_vel_b_error.z)
    grid on
    xlabel('time [sec]')
    ylabel('[m/s]')
    legend('x','y','z')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Hover Vel CMD
    
    figure('name','Hover Vel g CMD')
    plot(c.time, c.hover.wing_vel_g_cmd.x,'--')
    hold on
    plot(c.time, c.hover.wing_vel_g_cmd.y,'--')
    plot(c.time, c.hover.wing_vel_g_cmd.z,'--')
    grid on
    plot(c.time, c.state_est.Vg_f.x)
    plot(c.time, c.state_est.Vg_f.y)
    plot(c.time, c.state_est.Vg_f.z)
    xlabel('time [sec]')
    ylabel('[m/s]')
    legend('x vel cmd','y vel cmd','z vel cmd','x vel est','y vel est','z vel est')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Hover Angles
    figure('name','Hover Angles')
    plot(c.time,[c.hover.angles.x,c.hover.angles.y,c.hover.angles.z] * 180/pi,'-')
    hold on
    set(gca,'ColorOrderIndex',1)    % reset line color palette to original
    grid
    plot(c.time,[c.hover.angles_cmd.x,c.hover.angles_cmd.y,c.hover.angles_cmd.z] * 180/pi,'--')
    hold off
    title('Hover Attitudes')
    xlabel('time [sec]')
    ylabel('angle (deg)')
    legend('hover.angles.x','hover.angles.y','hover.angles.z','hover.angles\_cmd.x','hover.angles\_cmd.y','hover.angles\_cmd.z')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    
    %% Hover Tension
    figure('name','Hover Tension');
    plot(c.time,c.hover.tension_cmd,'--')
    hold on
    plot(c.time,c.hover.horizontal_tension_cmd,'--')
    plot(c.time,c.state_est.tether_force_b.tension_f)
    grid on
    title('Hover Tension')
    xlabel('time [sec]')
    ylabel('Newtons')
    legend('Tension cmd','Horizontal Tension cmd','Tension Est')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Hover Pitch
    figure('name','Hover Pitch')
    plot(c.time,c.hover.pitch_cmd*180/pi,'b--')
    hold on
    plot(c.time,c.hover.int_pitch*180/pi,'r-')
    plot(c.time,c.hover.angles.y*180/pi,'g-')
    grid on
    xlabel('time [sec]')
    ylabel('Degrees?')
    legend('hover.pitch\_cmd','hover.int\_pitch','hover.angles.y')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Hover Rate Errors
    try
        figure('name','Hover Rate Errors')
        perr = c.hover.pqr_cmd.x - c.state_est.pqr.x;
        qerr = c.hover.pqr_cmd.y - c.state_est.pqr.y;
        rerr = c.hover.pqr_cmd.z - c.state_est.pqr.z;
        plot(c.time,perr*180/pi);
        hold on
        plot(c.time,qerr*180/pi);
        plot(c.time,rerr*180/pi);
        legend('p err','q err','r err')
        xlabel('time [sec]')
        ylabel('DEG/SEC')
        grid on
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
    catch
    end
    
    
    %% Hover Position Command
    figure('name','Hover Position CMD')
    plot(c.time,c.hover.wing_pos_g_cmd.x,'--')
    hold on
    plot(c.time,c.hover.wing_pos_g_cmd.y,'--')
    plot(c.time,c.hover.wing_pos_g_cmd.z,'--')
    grid on
    plot(c.time,c.state_est.Xg.x)
    plot(c.time,c.state_est.Xg.y)
    plot(c.time,c.state_est.Xg.z)
    xlabel('time [sec]')
    ylabel('m')
    legend('x cmd','y cmd','z cmd')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Hover Angle Integrator
    figure('name','Hover Angle int')
    plot(c.time,c.hover.int_angles.x*180/pi,'b')
    hold on
    plot(c.time,c.hover.int_angles.y*180/pi,'g')
    plot(c.time,c.hover.int_angles.z*180/pi,'r')
    grid on
    xlabel('time [sec]')
    ylabel('[DEG]')
    legend('x','y','z')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
    
    %% Hover Moment Integrator
    figure('name','Hover Moment int')
    plot(c.time,c.hover.int_moment.x,'b')
    hold on
    plot(c.time,c.hover.int_moment.y,'g')
    plot(c.time,c.hover.int_moment.z,'r')
    grid on
    xlabel('time [sec]')
    ylabel('N*m')
    legend('x','y','z')
    timehistoryfigurelist = [timehistoryfigurelist; gcf];
end

%% GPS Plots
if IncludeGPSPlots
    try
        goidx = find(~c.state_est.gps_active);
        dgoidx = diff(goidx);
        wdgoidx = find(dgoidx>1);
        GPSout.startidx = [goidx(1); goidx(wdgoidx+1)];
        GPSout.endidx = [goidx(wdgoidx); goidx(end)];
        GPSout.duration = c.time(GPSout.endidx) - c.time(GPSout.startidx);
        
        BadGPSSol = (c.control_input.wing_gps.pos_sol_type(1,:)==1)&(c.control_input.wing_gps.pos_sol_type(2,:)==1);
        figure('name','GPS Pos Sol Type')
        plot(c.time,c.control_input.wing_gps.pos_sol_type(1,:),'bo')
        hold on
        plot(c.time,c.control_input.wing_gps.pos_sol_type(2,:),'r.')
        legend('1','2')
        title('GPS Sol Type')
        grid on
        xlabel('time [sec]')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
        figure('name','GPS Active')
        plot(c.time,c.state_est.gps_active,'o')
        grid on
        xlabel('time [sec]')
        ylabel('GPS Active')
        legend('GPS Active?')
        title('GPS Active')
        timehistoryfigurelist = [timehistoryfigurelist; gcf];
    catch
    end
end

%% Add Features to Each Plot
% List of axes
axlist = [];

if ~exist('in','var')
    in = [];
end

% Callback string for creating animations
animationcallbackstringwide = ['ax = axis;  figure(''name'',''Animation'');' ...
    ' animateidx = find((c.time>=ax(1))&(c.time<=(ax(2))));'...
    ' load m600geometry.mat; AnimateKite(c,rawm600,animateidx,in,0);'];


animationcallbackstringkite = ['ax = axis;  figure(''name'',''Animation'');' ...
    ' animateidx = find((c.time>=ax(1))&(c.time<=(ax(2))));'...
    ' load m600geometry.mat; AnimateKite(c,rawm600,animateidx,in,1);'];

% Loop through all figures
for linkidx = 1:numel(timehistoryfigurelist)
    
    % Make the figure active
    figure(timehistoryfigurelist(linkidx));
    
    % Add it to the list
    axlist = [axlist gca];
    
    %% Increase font size
    set(gca,'FontSize',14)
    
    %% Add animation menu
    animenu = uimenu(timehistoryfigurelist(linkidx),'Label','Animate');
    uimenu(animenu,'Label','Wide View','Callback', animationcallbackstringwide);
    uimenu(animenu,'Label','Kite View','Callback', animationcallbackstringkite);
end

%% Link time axes
linkaxes(axlist,'x')
