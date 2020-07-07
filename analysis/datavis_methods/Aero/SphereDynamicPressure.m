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

function pressure = ...
    SphereDynamicPressure(air_density, apparent_wind_p, port_direction_p)
%% Convert an apparent wind vector into a pressure at a pitot pressure port
% This function is based on SphereDynamicPressure in
% sim/models/sensors/pitot.cc
%
% Inputs:
%   - air_density: Air density in kg/m^3
%   - apparent_wind_p: Apparent speed vector in the pitot frame in m/s
%   - port_direction_p: Unit direction vector from the center of the
%                       hemispherical pitot probe to the pressure port of
%                       interest
%
% Returns:
%   - pressure: Pressure at the port of interest in Pa

%% Check that port_direction_p is a unit vector
assert(abs(1 - norm(port_direction_p)) < 1e-6);

%% Calculate the pressure
% The coefficient of pressure on a sphere is given by:
%
%   C_P = 1.0 - (9.0 / 4.0) * sin(theta)^2.
%
% We avoid trigonometric equations here by exploiting the fact that
% the cross product between vectors u and v is ||u|| * ||v|| *
% sin(theta), where theta is the angle between the vectors.  Since
% we only care about sin(theta)^2, we are not worried about the
% sense of the angle here. Additionally, ||u|| = 1.
%
% Hence:
%  C_P = 1.0 - (9.0 / 4.0) * (u x v)^2 / ||v||^2
%
% And P = 0.5 * rho * V_app^2 * C_P

pressure = 0.5 * air_density .* ...
    (MatVec3Norm(apparent_wind_p).^2 - ...
     (9 / 4) * MatVec3Norm(cross(apparent_wind_p, ...
                          port_direction_p * ...
                          ones(1, length(apparent_wind_p)))).^2);

end

function N = MatVec3Norm(M)
% Compute the L2-norm of a [3xn] matrix of [3x1] vectors.
% M = [vec1 vec2 ... vecn]
% N = [norm(vec1) norm(vec2) ... norm(vecn)]

N = sqrt(M(1, :).^2 + M(2, :).^2 + M(3, :).^2);

end