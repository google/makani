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

function [out] = ins_log_innovation(Pzz, dz, dz_avg, dx_plus, like2, valid, ...
                                    time, out, mm)
if mm <= length(out.innov) && ~isempty(out.innov{mm})
  i = length(out.innov{mm}.t) + 1;
else
  i = 1;
end
out.innov{mm}.t(i, :) = time;
out.innov{mm}.Pzz(i, :) = diag(Pzz)';
out.innov{mm}.dz(i, :) = dz';
out.innov{mm}.dz_avg(i, :) = dz_avg';
out.innov{mm}.dx_plus(i, :) = dx_plus';
out.innov{mm}.like2(i, :) = like2;
out.innov{mm}.valid(i, :) = valid;
