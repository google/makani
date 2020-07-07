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

function msub(rnum, cnum, subplot_num, subplot_name, ...
              xlabel_str, ylabel_str, zlabel_str, link_x_axes)
%  MSUB   Standard Makani subplot style
    if nargin < 8, link_x_axes = true; end
    if nargin < 7, zlabel_str = ''; end
    if nargin < 6, ylabel_str = ''; end
    if nargin < 5, xlabel_str = ''; end
    subplot(rnum, cnum, subplot_num);
    hold all;
    grid on;
    zoom('out');
    zoom('on');
    title(subplot_name);
    xlabel(xlabel_str);
    ylabel(ylabel_str);
    zlabel(zlabel_str);
    if link_x_axes
        evalin('caller', 'if ~exist(''ax''), ax=[]; end; ax(end + 1) = gca;');
    end
end
