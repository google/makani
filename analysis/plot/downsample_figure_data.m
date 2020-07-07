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

function downsample_figure_data(h, n)
%  DOWNSAMPLE_FIGURE_DATA downsample figure data for
%
%  Often this is useful prior to export to vector graphics file to keep
%  file size down.
%
%  h is a vector of line handles (output of the plot function)
%

for i=1:length(h)
    hi = h(i);
    set(hi, 'XData', downsample(get(hi, 'XData'), n));
    set(hi, 'YData', downsample(get(hi, 'YData'), n));
    set(hi, 'ZData', downsample(get(hi, 'ZData'), n));
end

