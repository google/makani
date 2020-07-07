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

function make_naming_shortcuts(data)
% make_naming_shortcuts --  Creates abbreviations for commonly used structures.
%
% make_naming_shortcuts(data)

try
  assignin('caller', 'C', data.ControllerA.ControlTelemetry);
  assignin('caller', 'C', data.ControllerA.ControlTelemetry);
end

try
  assignin('caller', 'c', data.ControllerA.ControlTelemetry.message)
  assignin('base', 'c', data.ControllerA.ControlTelemetry.message)
end

try
  assignin('caller', 'S', data.Simulator.SimTelemetry);
  assignin('base', 'S', data.Simulator.SimTelemetry);
end

try
  assignin('caller', 's', data.Simulator.SimTelemetry.message);
  assignin('base', 's', data.Simulator.SimTelemetry.message);
end

try
  assignin('caller', 'control_params', data.parameters.control_params);
  assignin('base', 'control_params', data.parameters.control_params);
end

try
  assignin('caller', 'system_params', data.parameters.system_params);
  assignin('base', 'system_params', data.parameters.system_params);
end

try
  assignin('caller', 'sim_params', data.parameters.sim_params);
  assignin('base', 'sim_params', data.parameters.sim_params);
end

end
