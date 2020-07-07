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

function plot_quartersphere(R)
% PLOT_QUARTERSPHERE plots a wire frame of a quartershere

n = 40;   % Will result in (n+1) by (n+1) sized matricies
nd2 = round(n/2);
zero = 1e-6;  % Use this as zero to prevent numerical error

[X, Y, Z] = sphere(n);

col = find(X(nd2,:) <= zero);
row = find(Z(:,nd2) <= zero);

% Prevent closing the backside of the quartersphere with patches
col = circshift(col, [0 round(n/4) + 1]);

X = X(row,col);
Y = Y(row,col);
Z = Z(row,col);
mesh(X*R, Y*R, Z*R, 'Edgecolor', [0.85 0.85 0.85], 'FaceColor', 'none');
