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

function [ apparent_wind ] = GetApparentWindVector( state_est )
  % GetApparentWindVector - apparent wind vector from M600 state estimator
  %
  % Uses the h5 file components: ControllerA and motor nodes
  % to calculate aerodynamic coefficients.
  %
  % Inputs:
  %   h5file_loc - the h5 flight database file
  %
  % Outputs:
  %   apparent_wind - [n x 3] array of u,v,w vs n time steps
  %
  % Important variables:
  %   state_est  - the estimated state of the kite read from the controller
  %
  %=====================================================================================================================
  %% Data Input from h5file
  %=====================================================================================================================
  % apparent wind vector in Body Coordinates
  u = state_est.apparent_wind.vector.x;
  v = state_est.apparent_wind.vector.y;
  w = state_est.apparent_wind.vector.z;
  apparent_wind = [u, v, w];
end

