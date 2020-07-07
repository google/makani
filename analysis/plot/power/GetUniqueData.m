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

function s = GetUniqueData(filename, nodestr, kMessageType)
% GetUniqueData -- Read h5 data, sort, and remove duplicates before returning.
%
% s = GetUniqueData(filename, nodestr, kMessageType)
% Extract data from H5 file by node name.  Uniquify data for good measure.  More
% recent versions of pcap converter may make this last step unnecessary.
%
% Arguments
%
% filename: Name of H5 file with flight data.
% nodestr: Name of node with 'kAioNode' removed.
% kMessageType: Aio message type with motor data
%
% Return values
%
% s: Structure with extracted data
%
% Required toolboxes: None.


  try
    if (nargin < 3)
      s = h5read(filename, nodestr);
    else
      s = h5read(filename, ['/messages/kAioNode', nodestr, '/', kMessageType]);
    end
    s = UniqueStruct(s);
  catch me
    disp(me.getReport)
    s.null = [];
  end
end


% Note that for large enough data sets this method of determining
% uniqueness will fail.  Also, uniqueness is now handled in the pcap to H5
% converter and may no longer be critical here.
function data = UniqueStruct(data)
  % UniqueStruct -- Return unique and sorted version of the structured data

  % First get the local steps so we can build a long int sequence
  seqstep = diff(double(data.aio_header.sequence));

  % Clean up large jumps -- sensitive to large.
  seqstep = seqstep + (seqstep < -32768) * 65536 - ...
            (seqstep > 32768) * 65536;

  % Build new sequence number
  seq = [0;cumsum(seqstep)];

  % Sort and uniquify
  [counter, ii, ~] = unique(seq);

  % Now traverse the structure if data is out of order or not unique
  if ~all(diff(ii) == 1)
    data = SubUnique(data, ii);
  end

  % Add new field with extended sequence number
  data.counter = counter;
end


function s = SubUnique(s, ii)
% SubUnique -- Rebuild data structure recursively
  if (isstruct(s))
    for fn = fieldnames(s)'
      s.(fn{1}) = SubUnique(s.(fn{1}), ii);
    end
  else
    if (size(s,2) ~= 1)
      s = s(:,ii);
    else
      s = s(ii);
    end
  end
end
