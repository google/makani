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

function S = extractBetweenStrings(STR, START_STR, END_STR)
% extractBetweenStrings -- extract substrings between indicators that mark
%                          starts and ends of substrings.
% newStr = extractBetweenStrings(STR, START_STR, END_STR)
%
% Extracts the substring from STR that occurs between the substrings START_STR
% and END_STR. The extracted substring does not include START_STR and END_STR.
% Limited replication of 'extractBetween' MATLAB function introduced in R2016b.
%
% Arguments-
% STR:       [char] string array or a cell array of character vectors.
% START_STR: [char] start string.
% END_STR:   [char] end string.
%
% Output Values-
% S:         [char] extracted substring between START_STR and END_STR.
%

% Convert string array to cell.
if ~iscell(STR)
  STR = num2cell(STR, 2);
end

% Loop through each cell element and extract between specified strings.
for ii = 1:length(STR)
  S1 = strsplit(STR{ii}, START_STR);
  S2 = strsplit(S1{2}, END_STR);
  S{ii} = S2{1};
  clear S1 S2;
end
clear ii;