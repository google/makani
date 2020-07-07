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

% processFltData.m

%% Specify log data file name
if exist('in','var')
    warning('The variable "in" is already in the workspace.\n Not loading new data.')
else
    % This is the .h5 file which is loaded into the workspace
    makani_home = getenv('MAKANI_HOME');
    logDataFileName = [makani_home '/logs/last.h5']
    
    %% Load the binary data file
    in = load_h5log(logDataFileName);
end

%% Pull out the ControllerA data
% (this is the main control system data)
% (See ~/makani/control/control_telemetry.h for data defs)
try
    if (isempty(in.ControllerA.ControlDebug))   % no debug data, so use std tlm msg
        c = in.ControllerA.ControlTelemetry.message;
    else
        c = in.ControllerA.ControlDebug.message; %<--- USE THIS
    end
catch
    c = in.ControllerA.ControlTelemetry.message;
end

%% Constants
r2d = 180.0 / pi;
d2r = 1.0 / r2d;

%% Define time = 0 and reassign c.time
time0 = c.time(min(find(c.flight_mode==2)));
try
    c.time = c.time - time0;
catch
end

%% Tether Radius
TetherRadius = sqrt(c.state_est.Xg.x.^2+c.state_est.Xg.y.^2+c.state_est.Xg.z.^2);

%% Configuration Number
% Once upon a time we had these
try
    c.confignum = double(c.state_est.experimental_crosswind_config).*double(c.crosswind.experimental_config_active);
catch
    c.confignum = -1*ones(size(c.time));
end

%% Power
try
    try
        motor_data = ProcessMotorData(logDataFileName);
        power = motor_data.total_bus_power;
    catch
        stime = in.Simulator.SimTelemetry.message.time;
        ptime = c.time(1) + stime - stime(1);
        power = in.Simulator.SimTelemetry.message.power_sys.P_elec;
        c.power = interp1(stime,power,c.time);
    end
catch
    [powers,ptime] = GetMotorPowers(in);
    ptime = ptime - ptime(1) + c.time(1);
    power = sum(powers,1);    
    c.power = interp1(ptime,power,c.time);
end

%% Assign parameters to c
c.parameters = in.parameters;

%% Test Cases
try
    c.testcase = int32(c.state_est.experimental_crosswind_config).*int32(c.state_est.joystick.pitch_f<-0.5);
catch
    c.testcase = 0*c.time;
end

Tens = c.state_est.tether_force_b.tension_f;
phitlist = c.state_est.tether_force_b.sph.roll;
thetatlist = c.state_est.tether_force_b.sph.pitch;
cwradius = sqrt(c.crosswind.current_pos_cw.x.^2+c.crosswind.current_pos_cw.y.^2);

%% Calculate angle based upon position
ang = pi-atan2(c.crosswind.current_pos_cw.y,-c.crosswind.current_pos_cw.x);

dang = diff(ang);
lidx = find(dang>6);

%% Calculate loop number array
try
    loopnum = (cumsum([0;diff(c.crosswind.loop_angle)]>6)+(c.flight_mode>=7) - 1).*((c.flight_mode==7)|(c.flight_mode==8));
    ln = unique(loopnum(loopnum>0));
    
    c.Loops = [];
    
    %% Populate A Structure With Per Loop Averages
    for idx = 1:numel(ln)
        c.Loops.num(idx) = idx;
        thisidx = find(loopnum==idx);
        c.Loops.idx{idx} = {thisidx};
        c.Loops.alpha_cmd(idx) = mean(c.crosswind.alpha_cmd(thisidx));
        c.Loops.alpha(idx) = mean(c.state_est.apparent_wind.sph_f.alpha(thisidx));
        c.Loops.beta_cmd(idx) = mean(c.crosswind.beta_cmd(thisidx));
        c.Loops.beta(idx) = mean(c.state_est.apparent_wind.sph_f.beta(thisidx));
        c.Loops.betamin(idx) = min(c.state_est.apparent_wind.sph_f.beta(thisidx));
        c.Loops.betamax(idx) = max(c.state_est.apparent_wind.sph_f.beta(thisidx));
        try
            c.Loops.confignum(idx) = mode(confignum(thisidx));
        catch
        end
        c.Loops.airspeed_cmd(idx) = mean(c.crosswind.airspeed_cmd(thisidx));
        c.Loops.airspeed(idx) = mean(c.state_est.apparent_wind.sph_f.v(thisidx));
        c.Loops.maxomega(idx) = max( max(abs(c.control_input.rotors(:,thisidx))));
        c.Loops.power(idx) = mean(c.power(thisidx));
        c.Loops.ail8satperc(idx) = numel( find((c.control_output.flaps(6,thisidx)>(9.75*pi/180))) ) /numel(thisidx);
        
        c.Loops.tetherrollcmdmax(idx) = max(c.crosswind.tether_roll_cmd(thisidx));
        c.Loops.tetherrollcmdmin(idx) = min(c.crosswind.tether_roll_cmd(thisidx));
        c.Loops.tetherrollcmd(idx) = mean(c.crosswind.tether_roll_cmd(thisidx));
        
        c.Loops.tetherradiusmin(idx) = min(TetherRadius(thisidx));
        c.Loops.tetherradiusmax(idx) = max(TetherRadius(thisidx));
        c.Loops.tensionmin(idx) = min(c.state_est.tether_force_b.tension_f(thisidx));
        c.Loops.tensionmax(idx) = max(c.state_est.tether_force_b.tension_f(thisidx));
        
        c.Loops.hubheight(idx) = -mean(c.crosswind.path_center_g.z(thisidx));
        c.Loops.radius(idx) = mean(sqrt(c.crosswind.current_pos_cw.x(thisidx).^2 ...
            + c.crosswind.current_pos_cw.y(thisidx).^2));
        try
            c.Loops.radius_cmd(idx) = mean(c.crosswind.path_radius_target(thisidx));
            c.Loops.radius_error(idx) = trapz(-c.crosswind.loop_angle(thisidx),   abs(c.crosswind.path_radius_target(thisidx)-cwradius(thisidx))  )/2/pi;
        catch
            c.Loops.radius_cmd(idx) = 0;
            c.Loops.radius_error(idx) = 0;
        end
        c.Loops.wind_speed(idx) = mean(c.state_est.wind_g.speed_f(thisidx));
        try
            c.Loops.windaloftspeed(idx) = mean(c.state_est.wind_aloft_g.speed_f(thisidx));
        catch
        end
        c.Loops.wind_dir(idx) = mean(c.state_est.wind_g.dir_f(thisidx));
    end
catch
end
