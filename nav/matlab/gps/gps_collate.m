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

function [eph, obs] = gps_collate(eph, obs, valid)
if ~iscell(eph)
  eph = array2cell(eph);
end
if ~iscell(obs)
  obs = array2cell(obs);
end
if exist('valid', 'var')
  obs = obs(valid(1:length(obs)) == 1);
end

% Remove empty cells.
eph = eph(cellfun('isempty', eph) == 0);
obs = obs(cellfun('isempty', obs) == 0);

% Select intersection of PRNs.
prns = intersect(cellfun(@(x) x.prn, eph), cellfun(@(x) x.prn, obs));
eph = eph(cellfun(@(x) ismember(x.prn, prns), eph) == 1);
obs = obs(cellfun(@(x) ismember(x.prn, prns), obs) == 1);

% Collate.
[~, ii] = sort(cellfun(@(x) x.prn, eph));
eph = eph(ii);
[~, ii] = sort(cellfun(@(x) x.prn, obs));
obs = obs(ii);

% Remove low quality observables.
if ~isempty(obs)
  ii = find(cellfun(@(x) x.qual, obs) >= 4);
  eps = eph(ii);
  obs = obs(ii);
end

% Remove unhealthy satellites.
if ~isempty(eph)
  ii = find(cellfun(@(x) x.health, eph) == 0);
  eph = eph(ii);
  obs = obs(ii);
end


function [c] = array2cell(a)
c = {};
for i = 1:length(a)
  c{i} = a(i);
end
