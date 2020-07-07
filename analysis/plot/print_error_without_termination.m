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

function print_error_without_termination()
% PRINT_ERROR_WITHOUT_TERMINATION prints last error without terminating
% execution

le = lasterror();
if ~isempty(le.message)
    fprintf(['\nThere were errors during plot_control:\n' ...
        'message: %s\nfile: %s\nname: %s\nline: %s\n'], ...
        le.message, le.stack(end).file, le.stack(end).name, ...
        num2str(le.stack(end).line));
end