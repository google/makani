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

function make_pdf_figure_book(file_name)
%  MAKE_FIGURE_BOOK saves all of the open figures into a single, multi-page pdf
%
%  file_name is the base filename of the pdf
%
%  Ex:
%  % Leave open all of the figures you want in the book
%  >> make_figure_book('figure_book')
%  % figure_book.pdf will appear in your current directory
% 
%  TODO: should have a feature to decimate data so that pdf's of long
%  data runs require less memory

% find the handles of all the open figures
h = findobj('Type', 'figure');
h = sort(h);

hs = struct('dateformat', 'none', ...
    'string', '', ...
    'fontname', 'Times', ...
    'fontsize', 12, ...  % in points
    'fontweight', 'normal', ...
    'fontangle', 'normal', ...
    'margin', 72);  % in points
        
for i=1:length(h);
    hs.string = get(h(i), 'Name');
    setappdata(h(i), 'PrintHeaderHeaderSpec', hs);
    print(h(i), '-dpsc2', '-append', file_name);
end

ps_file_name = [file_name '.ps'];
[status, result] = system(['ps2pdf ' ps_file_name]);
[status, result] = system(['rm ' ps_file_name]);

