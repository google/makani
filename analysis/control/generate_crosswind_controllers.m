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

function [models, controllers] = generate_crosswind_controllers()
  % Generates the crosswind controllers for the kite configurations.

  % NOTE: Because of changes in MATLAB's pidtune algorithm, this script
  % must be run using release r2014a for consistent results.
  assert(strcmp(version('-release'), '2014a'), ...
      'This script must be run with MATLAB 2014a.');

  wing_models = {'m600', 'oktoberkite'};
  for i = 1:length(wing_models)
    wing_model = wing_models{i};
    output_dir =  [getenv('MAKANI_HOME'), '/config/', wing_model, '/control/'];

    % Serials are in struct WingSerialToString in control/system_types.c.
    switch wing_model
      case 'm600'
        wing_serials = {'kWingSerial01', 'kWingSerial04Hover', ...
                        'kWingSerial04Crosswind', 'kWingSerial05Hover', ...
                        'kWingSerial05Crosswind', 'kWingSerial06Hover', ...
                        'kWingSerial06Crosswind', 'kWingSerial07Hover', ...
                        'kWingSerial07Crosswind'};

      case 'oktoberkite'
        wing_serials = {'kWingSerialOktoberKite01'};

        otherwise
        error(['Wing model ', wing_model, ' not recognized.']);
    end

    for k = 1:length(wing_serials)

      params{k} = get_parameters(wing_model, wing_serials{k});

      % Model the motors with a single pole.
      models{k}.motors = tf([2 * pi * params{k}.motor_bandwidth_hz], ...
                            [1, 2 * pi * params{k}.motor_bandwidth_hz]);

      % Make an airspeed model and generate a simple PID controller.
      models{k}.airspeed = calc_airspeed_model(params{k});
      controllers{k}.airspeed = safe_pidtune( ...
          models{k}.airspeed * models{k}.motors, 0.3, 80);
    end

    write_controllers([output_dir, 'crosswind_airspeed_controller.py'], ...
                      wing_serials, controllers);
  end
end

% Make a model that represents the airspeed dynamics.  This is
% actually a model of the inertial speed dynamics, but these values
% are close enough that we assume the same model applies.  The model
% is based around linearizing the speed dynamics about the
% steady-state speed, which is the lift-to-drag ratio multiplied by
% the wind speed:
%
%   v = (C_L/C_D) * v_w
%
% There is an effective restoring force for any perturbations away
% from this value described by k_eff in the equations below.  This
% effective restoring force depends on wind speed; however, for now we
% simply choose a representative wind speed.
%
% TODO: Prove stability about full range of wind speeds
% and account for gravity.
function model = calc_airspeed_model(params)
  wind_speed = 10;
  k_eff = 0.5 * params.air_density * params.wing_area * params.C_L_0 * ...
          wind_speed;
  model = tf([1], [params.effective_mass, k_eff]);
end

function params = get_parameters(wing_model, wing_serial)
  if strcmp(wing_model, 'm600')
    wing_area = 32.9;
    wing_CL0 = 1.99;
    switch wing_serial
      case 'kWingSerial01'
        wing_mass = 1648.2 + 55.3 + 10.6 + 0.6;

      case 'kWingSerial04Hover'
        wing_mass = 1746.4 + 52.1 + 10.6;

      case 'kWingSerial04Crosswind'
        wing_mass = 1667.5 + 52.1 + 10.6 + 0.6;

      case 'kWingSerial05Hover'
        wing_mass = 1711.8 + 51.5 + 10.6;

      case 'kWingSerial05Crosswind'
        wing_mass = 1629.6 + 51.5 + 10.6 + 0.6;

      case 'kWingSerial06Hover'
        wing_mass = 1750.2 + 51.5 + 10.6;

      case 'kWingSerial06Crosswind'
        wing_mass = 1667.9 + 51.5 + 10.6 + 0.6;

      case 'kWingSerial07Hover'
        % TODO(b/145244788): The following data pertain to SN04. Update.
        wing_mass = 1746.4 + 52.1 + 10.6;

      case 'kWingSerial07Crosswind'
        % TODO(b/145244788): The following data pertain to SN04. Update.
        wing_mass = 1667.5 + 52.1 + 10.6 + 0.6;

      otherwise
        error('Unrecognized wing serial %s for wing model: %s.', ...
            wing_serial, wing_model);
    end
  elseif strcmp(wing_model, 'oktoberkite')
    wing_area = 54.0;
    wing_CL0 = 1.99;
    switch wing_serial
      case 'kWingSerialOktoberKite01'
        wing_mass = 1850.0;
      otherwise
        error('Unrecognized wing serial %s for wing model: %s.', ...
            wing_serial, wing_model);
    end
  else
    error('Unrecognized wing model: %s.', wing_model);
  end

  params = struct('wing_area', wing_area, ...
                  'air_density', 1.2, ...
                  'effective_mass', wing_mass + 390.5 / 3, ...
                  'motor_bandwidth_hz', 6, ...
                  'C_L_0', wing_CL0);
end

function write_controllers(filename, wing_serials, controllers)
  fid = fopen(filename, 'w');
  fprintf(fid, ...
          ['"""Automatically generated crosswind controllers.', ...
           '\n', ...
           '\nThis file was generated by: ', ...
           'analysis/control/generate_crosswind_controllers.m.', ...
           '\n"""', ...
           '\n', ...
           '\nfrom makani.control import control_types as m', ...
           '\n', ...
           '\n', ...
           '\ndef GetControllers(wing_serial):', ...
           '\n  """Returns PID gains for crosswind airspeed controllers."""']);

  for k = 1:length(wing_serials)
    if k == 1
      fprintf(fid, '\n  if wing_serial == m.%s:', wing_serials{k});
    else
      fprintf(fid, '\n  elif wing_serial == m.%s:', wing_serials{k});
    end

    write_pid_controller(fid, '    ', 'airspeed', controllers{k}.airspeed);
  end

  fprintf(fid, ['\n  else:', ...
                '\n    assert False, ''wing_serial %%d was not ', ...
                'recognized'' %% wing_serial']);

  fprintf(fid, ...
          ['\n', ...
           '\n  return {', ...
           '\n      ''airspeed'': airspeed,', ...
           '\n  }\n']);
  fclose(fid);
end

function write_pid_controller(fid, indentation, name, controller)
  fprintf(fid, ['\n', indentation, name, ' = {', ...
                '\n', indentation, '    ''kp'': %#0.3g,', ...
                '\n', indentation, '    ''ki'': %#0.3g,', ...
                '\n', indentation, '    ''kd'': %#0.3g', ...
                '\n', indentation, '}'], ...
          controller.Kp, controller.Ki, controller.Kd);
end
