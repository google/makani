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

function s = cat_data_files(file_patterns, slice_struct)
%  CAT_DATA_FILES   Concatenate full or partial data structures accross
%   multiple files.
%
%   The filename input can be a string or cell array and can use the
%   same regexp vocabulary used on the linux command prompt (e.g. the
%   wildcards * or [1-3])
%   (TODO: windows not yet tested)
%
%   TODO: leverage the load_h5logs() options to pick particular fields
%   out of the data
%
%   Ex: concatonate two files from a flight
%       data = cat_datafiles('/logs/20130418-18[12]*a_good_flight.h5')
%
%   Ex: extract only wind over a twenty-four hour period
%       >> s.A.packet.control.time = []
%       >> s.A.packet.control.state_est.wind_g = []
%       >> wind_data = cat_datafiles('/logs/20130418*.h5',s)

if ~exist('slice_struct','var') || isempty(slice_struct)
    use_all_signals = true;
else
    use_all_signals = false;
    s = slice_struct;
end

fnames = ls_cell_array(file_patterns);

for i = 1:length(fnames)
    disp(['Loading...  ', fnames{i}]);
    data = load_h5log(fnames{i});
    if i == 1 && use_all_signals
        s = data;
    else
        s = cat_struct(s, data);
    end
end


function fnames = ls_cell_array(regex_input)
% Create a list of filenames using regular expressions in a string or
% cell array.  On Linux, it uses "ls", so the Unix command prompt regexp
% vocabulary is supported.
%
% The input may be a single regular expression or a cell array of regular
% expressions.

% Make able to accept string or cell array input
if isa(regex_input, 'char')
    regex{1} = regex_input;
else
    regex = regex_input;
end

fnames = [];
for i = 1:length(regex)
    % TODO: suspect will need a isunix, ispc elseif statement here
    [status, ls_out] = system(['ls -1 ' regex{i}]);
    if status ~= 0
        error(ls_out);
    end
    fnames = [fnames cellstr_ls_output(ls_out)];
end


% cellstr doesn't work on the output of "ls", so make my own
function c = cellstr_ls_output(ls_out)
I = find(ls_out == sprintf('\n'));
I = [1, I+1];  % Starting index of each filename

for i = 1:length(I)-1
    c{i} = ls_out(I(i):I(i+1)-2);
end
