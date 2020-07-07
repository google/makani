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

function dx = DiffLpf(x, fs, fc)
  % Description:
  % - Apply a first order low-pass filter of cut-off frequency fc
  % - Differentiate the filtered signal wrt to time
  %
  % Input:
  %   - x     raw signal
  %   - fs    sampling frequency (Hz)
  %   - fc    LPF cut-off frequency (Hz)
  %
  % Output:
  %   - z
  %
  % Example:
  %   dt = 0.1;
  %   t = 0:dt:10;
  %   x = cos(2*pi*5.*t) + rand(1, length(t));
  %   z = DiffLpf(x, 1/dt, 10)
  %
  %
  %%
  %
  % N = length(x);
  % y = zeros(N, 1);
  % z = zeros(N, 1);
  %
  %%
  dt = 1/fs;
  %
  df = designfilt('differentiatorfir','FilterOrder',50,...
                  'PassbandFrequency',fc,'StopbandFrequency',fc*1.2,...
                  'SampleRate',fs);
  %
  D = mean(grpdelay(df)); % filter delay
  dx = filter(df,[x; zeros(D,1)]);
  dx = dx(D+1:end);
  dx = dx/dt;
  %
  %
  %% First order LPF
  % alpha = 2*pi*dt*fc/(2*pi*dt*fc+1);
  % y(1) = x(1);
  % for n = 2:N
  %     y(n) = alpha*x(n) + (1-alpha)*y(n-1);
  % end
  %
  %% Central-difference
  % z(1) = (y(2)-y(1))/dt;
  % for n = 2:N-1
  %     z(n) = (y(n+1)-y(n-1))/(2*dt);
  % end
  % z(N) = (y(N)-y(N-1))/dt;
end
