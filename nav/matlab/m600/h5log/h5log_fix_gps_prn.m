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

function [obs] = h5log_fix_gps_prn(obs, prn)
num_sats = size(prn, 1);
num_recs = size(prn, 2);
prns = unique(prn(:));
prns(prns == 0) = [];

fields = fieldnames(obs);
for f = 1:length(fields)
  field = fields{f};
  if size(obs.(field), 1) == num_sats
    result = zeros(length(prns), num_recs);
    for p = 1:length(prns)
      F = find(prn == prns(p));
      [~, j] = ind2sub(size(prn), F);
      I = sub2ind(size(result), repmat(p, length(j), 1), j);
      result(I) = obs.(field)(F);
    end
    obs.(field) = result;
  end
end
obs.prn_list = prns;