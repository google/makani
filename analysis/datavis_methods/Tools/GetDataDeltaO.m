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

function [ data_order ] = GetDataDeltaO( data_min, data_max )
  %GetDataDeltaO Obtains the relevant order of a range of data for nice plotting purposes
  %
  % Usage:
  %   [ data_order ] = GetDataDeltaO( data_min, data_max )
  %
  % Description:
  %   This function will return the seemingly most relevant order of the data range given
  %
  % Inputs:
  %   data_min  - the minimum value of the data
  %   y_data    - the maximum value of the data
  %
  % Outputs:
  %   data_order is the seemingly most relevant order of the data range
  %
  %%====================================================================================================================
  raw_delta  = data_max - data_min;
  % get most significant order of the data
  order_max   = 10^ceil(log10(abs(data_max)));
  order_min   = 10^floor(log10(abs(data_min)));
  data_order  = max([order_max; order_min]);
  %
  order_delta = 10^floor(log10(abs(raw_delta)));
  % control the small data spreads vs. order of data
  while(data_order>=order_delta)
    data_order=data_order/10;
  end
end

