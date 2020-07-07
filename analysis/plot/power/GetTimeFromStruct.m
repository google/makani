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

function [t, t0] = GetTimeFromStruct(s, offset, tdiff, bFixTime)
% GetTimeFromStruct -- Extract time from structure using aio or capture time
%
% [t, t0] = GetTimeFromStruct(s, offset, tdiff)
% Extract time from flight data struct using either aio node sequence numbers or
% capture header data.  Depending on the communication method or the archiving
% computer, either method may be preferable in some cases.
%
% Arguments
%
% s: Structure with extracted h5 data.
% offset: Either string 'raw' to get data that doesn't have t0 stripped off,
%         otherwise if zero then set offset to t0, else use offset.
% tdiff: If 0 then use capture header, otherwise time = counter * tdiff
% bFixTime: Boolean whether to clean up capture header time with linear fit
%
% Return values
%
% t: vector of time values.
% t0: Return absolute time used to set zero relative time.
%
% Required toolboxes: None.

if (nargin < 4)
  bFixTime = false;
end

if (nargin < 3)
    tdiff = 0; % 'capture_header'
end

if (nargin < 2)
    offset = 0;
end

% Assume input of zero implies capture header and otherwise it is the time
% base for the message
if (tdiff < eps)
    if isfield(s, 'capture_header')
      if (ischar(offset) && strcmp(offset, 'raw'))
        offset = 0;
      elseif offset == 0
        offset = double(s.capture_header.tv_sec(1)) + ...
                 double(s.capture_header.tv_usec(1)) * 1e-6;
      end

      t0 = offset;

      t = double(s.capture_header.tv_sec) + ...
             (double(s.capture_header.tv_usec) * 1e-6) - offset;

      % If requested, clean time up
      if (bFixTime)
        t = FixTime(s.counter, t);
      end
    else
      disp('No capture_header available');
      t = offset;
      t0 = offset;
    end
else
    if nargin < 2
        t0 = 0;
    else
        t0 = offset;
    end

    if isfield(s, 'counter')
      t = double(s.counter) * tdiff + t0;
    else
      t = t0;
    end
end

end

function [new_time] = FixTime(counter, time)
% FixTime - Quick hack to clean up capture time noise
% Doesn't deal with clock drift on source side

p = polyfit(counter, time, 1);

new_time = polyval(p, counter);
end

