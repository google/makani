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

function [] = SetMatlab()
% SetMatlab -- Configures MATLAB's environment for Makani's tools.
%
% Sets up MATLAB's environment for use with our analysis and plotting scripts by
% adding the following directories to MATLAB's path:
%
%     analysis/dyno
%     analysis/motor
%     analysis/plot
%     analysis/util
%     analysis/derived_data
%
  makani_home = getenv('MAKANI_HOME');
  if (isempty(makani_home))
    % If MAKANI_HOME was never set in the terminal that launched Matlab or if
    % Matlab was started outside of a terminal, the string makani_home will be
    % empty and MAKANI_HOME needs to be derived. The path of SetMatlab is used
    % here (as opposed to other implementations such as just calling pwd()) so
    % that SetMatlab can be called from startup scripts in other directories of
    % the file system.
    makani_home = mfilename('fullpath');
    makani_home = makani_home(1:(end - length('/analysis/SetMatlab')));
    setenv('MAKANI_HOME', makani_home);
  end

  addpath([makani_home, '/analysis/dyno']);
  addpath([makani_home, '/analysis/motor']);
  addpath([makani_home, '/analysis/plot']);
  addpath([makani_home, '/analysis/plot/power']);
  addpath([makani_home, '/analysis/util']);
  addpath([makani_home, '/analysis/derived_data']);
end
