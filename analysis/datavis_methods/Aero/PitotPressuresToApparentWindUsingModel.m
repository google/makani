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

function [apparent_wind_p] = PitotPressuresToApparentWindUsingModel(alpha_press, beta_press, dyn_press, rho, port_angle)
% PitotToApparentWind -- Convert 5-port probe pressures to apparent wind.
%
% [apparent_wind] = PitotPressuresToApparentWindUsingModel(alpha_press, ...
%                                        beta_press, dyn_press, rho, port_angle)
%
% Arguments:
%
%   alpha_press: Differential pressure bottom minus top [Pa].
%   beta_press:  Differential pressure starboard minus port [Pa].
%   dyn_press:   Differential pressure center hole minus static ring [Pa].
%   rho:         Air density [kg/m^3].
%   port_angle:  Theoretical angle [rad] between the axis of symmetry of
%                the Pitot tube and its off-axis ports.
%                See makani/config/m600/pitot.py.
%
% All arguments except port_angle must be row vectors of the same length.
%
% Return value:
%
%    apparent_wind: Column vector of x, y, z components of apparent wind in
%                   the pitot frame [m/s].

%   Extracted from makani/control/sensor_util.c:
%
%   // The current Pitot tube has a hemispherical tip, and for small
%   // angles the expected pressure differences measured between the
%   // Pitot ports can be approximated by the pressure distribution on a
%   // sphere giving:
%   //
%   // dyn_press =
%   //     dynamic_pressure * (1 - 9/4 * (1 - cos(alpha)^2 * cos(beta)^2)),
%   // alpha_press =
%   //     dynamic_pressure * (9/2) * sin(2 * port_angle) * cos(beta)^2
%   //     * cos(alpha) * sin(alpha),
%   // beta_press =
%   //     dynamic_pressure * (9/2) * sin(2 * port_angle) * cos(alpha)
%   //     * cos(beta) * sin(beta).
%   //
%   // The wind vector has components:
%   //
%   //   u = V * cos(alpha) * cos(beta),
%   //   v = V * sin(beta),
%   //   w = V * cos(beta) * sin(alpha).
%   //
%   // If we write c = rho * 9/4 * sin(2 * port_angle), then:
%   //
%   //   a = alpha_press / c = u * w
%   //   b = beta_press / c = u * v
%   //   d = dyn_press / (0.5 * rho) = u^2 - 5/4 * (v^2 + w^2)
%   //                               = u^2 - 5/4 * (a^2 + b^2) / u^2.
%   //
%   // We can thus solve a quadratic equation for u^2, and then use this
%   // to find v and w.

c = rho * (9.0 / 4.0) * sin(2.0 * port_angle);
a = alpha_press ./ c;
b = beta_press ./ c;
d = dyn_press ./ (0.5 * rho);

apparent_wind_p_x = -sqrt(0.5 * (d + sqrt(d.^2 + 5.0 * (a.^2 + b.^2))));
apparent_wind_p_y = b ./ apparent_wind_p_x;
apparent_wind_p_z = a ./ apparent_wind_p_x;

apparent_wind_p = [apparent_wind_p_x; apparent_wind_p_y; apparent_wind_p_z];