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

function  [ alpha_deg beta_deg ] =  GetAlphaBetaFromEstimator(state_estimator)
  % GetAlphaBetaFromEstimator - alpha and beta angles from M600 state estimator
  %
  % Uses state estimator to get apparent wind and decomposes u,v,w into
  %   alpha and beta angles
  %
  % Inputs:
  %   state_est  - the estimated state of the kite read from the controller
  %
  % Outputs:
  %   alpha_deg - [n x 3] array of u,v,w vs n time steps
  %
  apparent_wind = GetApparentWindVector( state_estimator );
  apparent_wind_mag = (apparent_wind(:,1) .^ 2 + apparent_wind(:,2) .^ 2 + apparent_wind(:,3) .^ 2) .^ 0.5;
  %
  alpha_rad = atan2(-apparent_wind(:,3),-apparent_wind(:,1));
  alpha_deg = rad2deg(alpha_rad);
  %
  beta_rad = asin(-apparent_wind(:,2) ./ apparent_wind_mag);
  beta_deg  = rad2deg(beta_rad);
end

