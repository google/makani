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

function [density] = calculate_density(pressure, temperature, humidity)
% calculate_density -- Calculate the air density from measured air properties.
% [density] = calculate_density(pressure, temperature, humidity)
% Refs: https://en.wikipedia.org/wiki/Density_of_air#Humidity_.28water_vapor.29
%
% Arguments --
%
% pressure:    [dbl] Array of measured total pressure [Pa]
% temperature: [dbl] Array of measured temperature [C]
% humidity:    [dbl] Array of measured relative humidity [%]
%
% Output Values --
%
% density: [dbl] Array of calculated air density [kg/m^3]

% Specific gas constant [J/(Kg.K)] of dry air and water vapor
R_d = 287.058;
R_v = 461.495;

% saturation vapor pressure [Pa]
p_sat = 6.1078*10.^(7.5*temperature./(temperature + 237.3))*100;

% partial pressure of vapor [Pa]
p_v = (humidity/100).*p_sat;

% partial pressure of dry air [Pa]
p_d = pressure - p_v;

% calcualted density [kg/m^3]
density = p_d./(R_d*(temperature + 273.15)) + ...
          p_v./(R_v*(temperature + 273.15));