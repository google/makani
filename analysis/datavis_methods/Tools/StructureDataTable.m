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

function [t] = StructureDataTable(flight_modes,crosswind_cases,d)
%StructureDataTable function to return a data table from given structure
% Usage:
%   [t] = StructureDataTable(flight_modes,crosswind_cases,d)
%   summary(t)
%   grpstats(t,'mode_case_names',{'mean','min','max'},'DataVars',{'tether_tension','airspeed'})
%   loop_tension_stats = grpstats(t,{'loop_count','mode_case_names'},{'min','max'},'DataVars',{'tether_tension'})
%   full_loop_tension_stats = loop_tension_stats((loop_tension_stats.min_loop_angle<0.01)&(loop_tension_stats.max_loop_angle>2*pi*0.99),:)  % this removes any partial loops in each case
%   loop_stats_all =  grpstats(t,{'loop_count','mode_case_names'},{'min','max'},'DataVars',{'loop_angle','tether_tension','tether_pitch','tether_roll'});
%   loop_stats.r_tether_tension = loop_stats.min_tether_tension./loop_stats.max_tether_tension;
%   loop_stats = loop_stats_all((loop_stats_all.min_loop_angle<0.01)&(loop_stats_all.max_loop_angle>2*pi*0.99),:);
%   loop_max_tension = grpstats(t.tether_tension,t.loop_count,{'max'})
%   boxplot(t.tether_tension,t.mode_case_names,'PlotStyle','compact')
%
% Description:
%   feed this the outputs from the get_modes_and_cases, and a structure of
%   variables you care about, and it outputos a Table with a few different
%   group options that you can then use the grpstats and other built-in
%   matlab functions for (most will require the statistics toolbox, which
%   is included in the google floating license)
%
% Inputs:
%   flight_modes:       from get_modes_and_cases
%   crosswind_cases:    from get_modes_and_cases
%   d:                  structure which has column vectors of data you want (any fields that aren't a column vector of the proper length will be ignored)
%
% Outputs:
%   t:        table, with the field names as the variable names, and also
%   options for use in grouping: fltModeNames, xwindCaseNames, and mode_case_names (for both combined)
%
% Future Work:
%   - figure out best way to have loop count also be a group-by option
%   (work-around is to feed it a loop_count variable in the structure, such
%   as "d.loop_count= cumsum([0;diff(d.loop_angle)]>6)+(d.fltMode>=7);"

%% BEGIN
%
datanames= fieldnames(d);

% Get the flight mode labels, specify rpx # for flights before rpx-08.
[~, fltModeDef] = get_flight_mode_labels;

% add name of mode to mode number
for ii= 1:length(flight_modes.all_modes)
%    extract number from all_modes and catenate
   all_modes_names(ii,1) = {[flight_modes.all_modes{ii}, ': ', ...
       fltModeDef{str2num(char(regexp(flight_modes.all_modes{ii},'\d*','Match')))+1}]};
   % name mode for each row
   s.fltModeNames(d.flight_modes.indices.(d.flight_modes.all_modes{ii})(1):...
       d.flight_modes.indices.(d.flight_modes.all_modes{ii})(2),:)= all_modes_names(ii);
end
datalength= length(s.fltModeNames);

% initialize to proper length
s.xwindCaseNames{datalength,1}=[];
xwindCaseNames{datalength,1}=[];
% if crosswind_cases.active
if ~isempty(crosswind_cases)
for ii= 1:length(crosswind_cases.all_cases)
%     name case for each row
   s.xwindCaseNames(crosswind_cases.indices.(crosswind_cases.all_cases{ii})(1):...
       crosswind_cases.indices.(crosswind_cases.all_cases{ii})(2),:)= ...
       {[crosswind_cases.all_cases{ii}]};
   xwindCaseNames(crosswind_cases.indices.(crosswind_cases.all_cases{ii})(1):...
       crosswind_cases.indices.(crosswind_cases.all_cases{ii})(2),:)= ...
       {[' - ',crosswind_cases.all_cases{ii}]};
end
% combine mode and case into single string
s.mode_case_names = strcat(d.fltModeNames,xwindCaseNames);
end

nodropped=1;
for ii=1:size(datanames)
    if size(d.(datanames{ii}),1) == datalength % only use if the right size
        s.(datanames{ii}) = d.(datanames{ii});
    else
        if nodropped
            warning('the size of some fields do not match full data and will not be included in the returned DataTable:')
            nodropped=0;
        end
        warning([datanames{ii}])
    end
end





t= struct2table(s); % convert from structure to table


end
%
%% END FUNCTION StructureDataTable