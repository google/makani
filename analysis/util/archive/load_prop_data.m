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

function prop_data = load_prop_data(filename)
%  LOAD_PROP_DATA   Reads a propeller database file into a MATLAB struct.
%   prop_data = LOAD_PROP_DATA(filename)
%
%   Useful definitions:
%       lambda = V / (omega * R)
%       J = V / (f * D) = pi * lambda
    if nargin < 1
        filename = [getenv('MAKANI_HOME'), '/database/m600/m600_rotor.dat'];
    end

    R = 0.78/2;  % [m]
    rho = 1.2;  % [kg/m^3]
    prop_data = struct('v_inf', [], 'omega', [], 'power', [], ...
        'thrust', [], 'eff', [], 'inflow', [], 'torque', [], ...
        'lambda', [], 'J', [], 'C_T', [], 'C_P', []);

    fid = fopen(filename, 'r');
    if fid < 0
        fprintf('Could not open file: %s', filename);
        return
    end

    dim = fscanf(fid, '%d', 2);
    omega = fscanf(fid, '%f', dim(1));
    v_inf = fscanf(fid, '%f', dim(2));

    v_inf = repmat(v_inf, 1, dim(1));
    omega = repmat(omega', dim(2), 1);

    power = fscanf(fid, '%f', [dim(2), dim(1)]);
    thrust = fscanf(fid, '%f', [dim(2), dim(1)]);
    eff = fscanf(fid, '%f', [dim(2), dim(1)]);
    inflow = fscanf(fid, '%f', [dim(2), dim(1)]);

    torque = power ./ omega;
    lambda = v_inf ./ (omega * R);
    J = lambda * pi;
    C_T = thrust ./ (0.5*rho*v_inf.^2 * pi*R^2 ./ lambda.^2);
    C_P = torque ./ (0.5*rho*v_inf.^2 * pi*R^3 ./ lambda.^2);

    prop_data.v_inf = v_inf;
    prop_data.omega = omega;
    prop_data.power = power;
    prop_data.thrust = thrust;
    prop_data.eff = eff;
    prop_data.inflow = inflow;
    prop_data.torque = torque;
    prop_data.lambda = lambda;
    prop_data.J = J;
    prop_data.C_T = C_T;
    prop_data.C_P = C_P;
end
