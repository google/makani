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

function plot_rotation_matrix(dcm_g2b, X, s)
% PLOT_ROTATION_MATRIX plots the three body axis of using matlab color order:
%   body x-axis: blue
%   body y-axis: green
%   body z-axis: red
%
%   INPUTS
%   dcm_g2b: 
%   X : location of the origin (default: [0 0 0]')
%   s : length of the body axis (default: 1)

if nargin < 2; X = [0 0 0]'; end
if nargin < 3; s = 1; end

if length(s) == 1
    s = [s s s];
end

if isempty(dcm_g2b) || isempty(X)
    return
end

R = dcm_g2b';

ax1 = [X, X + R(:,1)*s(1)];
ax2 = [X, X + R(:,2)*s(2)];
ax3 = [X, X + R(:,3)*s(3)];

plot3(ax1(1,:), ax1(2,:), ax1(3,:), 'b', ...
    ax2(1,:), ax2(2,:), ax2(3,:), 'g', ...
    ax3(1,:), ax3(2,:), ax3(3,:), 'r');

axis equal;
xlabel('x');
ylabel('y');
zlabel('z');


