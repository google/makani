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

function [struct_out] = transpose_my_struct(struct_in)
% transpose_my_struct -- recursively transposes all numeric arrays in the structure.
% [struct_out] = transpose_my_struct(struct_in)
%
% Arguments-
%
% struct_in: Input structure
%
% Return Values-
%
% struct_out: output structure with all numeric arrays transposed for h5 export.
%

% initialize the output
struct_out = struct_in;

% recursively find all the fieldnames in the structure
all_fields = fieldnamesr(struct_in);

% loop through each fieldname, and if numeric, transpose it
for ii = 1:numel(all_fields);
  if isnumeric (eval(['struct_in.' all_fields{ii,1}])) && iscolumn (eval(['struct_in.' all_fields{ii,1}]));
    eval (['struct_out.' all_fields{ii,1} '= transpose(struct_in.' (all_fields{ii,1}) ');']);
  end
end

% appending the third party fieldnamesr function

function NAMES = fieldnamesr(S,varargin)
%FIELDNAMESR Get structure field names in recursive manner.
%
%   NAMES = FIELDNAMESR(S) returns a cell array of strings containing the
%   structure field names associated with s, the structure field names
%   of any structures which are fields of s, any structures which are
%   fields of fields of s, and so on.
%
%   NAMES = FIELDNAMESR(S,DEPTH) is an optional field which allows the depth
%   of the search to be defined. Default is -1, which does not limit the
%   search by depth. A depth of 1 will return only the field names of s
%   (behaving like FIELDNAMES). A depth of 2 will return those field
%   names, as well as the field names of any structure which is a field of
%   s.
%
%   NAMES = FIELDNAMESR(...,'full') returns the names of fields which are
%   structures, as well as the contents of those structures, as separate
%   cells. This is unlike the default where a structure-tree is returned.
%
%   NAMES = FIELDNAMESR(...,'prefix') returns the names of the fields
%   prefixed by the name of the input structure.
%
%   NAMES = FIELDNAMESR(...,'struct') returns only the names of fields
%   which are structures.
%
%   See also FIELDNAMES, ISFIELD, GETFIELD, SETFIELD, ORDERFIELDS, RMFIELD.

%   Developed in MATLAB 7.13.0.594 (2011b).
%   By AATJ (adam.tudorjones@pharm.ox.ac.uk). 2011-10-11. Released under
%   the BSD license.

%Set optional arguments to default settings, if they are not set by the
%caller.
if size(varargin,1) == 0
    optargs = {-1} ;
    optargs(1:length(varargin)) = varargin ;
    depth = optargs{:} ;
    
    full = [0] ; [prefix] = [0] ; [struct] = [0] ;
else
    if any(strcmp(varargin,'full') == 1)
        full = 1 ;
    else
        full = 0 ;
    end
    
    if any(strcmp(varargin,'prefix') == 1)
        prefix = 1 ;
    else
        prefix = 0 ;
    end
    
    if any(strcmp(varargin,'struct') == 1)
        struct = 1 ;
    else
        struct = 0 ;
    end
    
    if any(cellfun(@isnumeric,varargin) == 1)
        depth = varargin{find(cellfun(@isnumeric,varargin))} ;
    else
        depth = -1 ;
    end
end


%Return fieldnames of input structure, prefix these with "S.".
NAMES = cellfun(@(x) strcat('S.',x),fieldnames(S),'UniformOutput',false) ;

%Set state counters to initial values (k terminates recursive loop, g makes
%recursive loop behave in a different way.
k = 1 ; g = 0 ;

fndstruct = {} ;

%k is ~0 while all fields have not been searched to exhaustion or to
%specified depth.
while k ~= 0
    fndtemp = {} ;
    
    k = length(NAMES) ;
    
    %g set to 1 prevents fieldnames from being added to output NAMES.
    %Useful when determining whether fields at the lowest specified depth
    %are structures, without adding their child fieldnames (i.e. at
    %specified depth + 1) to NAMES output.
    if depth == 1
        g = 1 ;
    end
    
    for i = 1:length(NAMES)
        %If the current fieldname is a structure, find its child
        %fieldnames, add to NAMES if not at specified depth (g = 0). Add to
        %fndstruct (list of structures). 
        if isstruct(eval(NAMES{i})) == 1
            if g ~= 1
                fndtemp2 = fieldnames(eval(NAMES{i})) ;
                fndtemp2 = cellfun(@(x) strcat(sprintf('%s.'            ...
                    ,NAMES{i}),x),fndtemp2,'UniformOutput',false) ;
                fndtemp = cat(1,fndtemp,fndtemp2) ;
            elseif g == 1
                fndtemp = cat(1,fndtemp,NAMES{i}) ;
                k = k - 1 ;
            end
            fndstruct = cat(1,fndstruct,NAMES{i}) ;
        else
            fndtemp = cat(1,fndtemp,NAMES{i}) ;
            k = k - 1 ;
        end
    end
    
    NAMES = fndtemp ;
    
    %If we have reached depth, stop recording children fieldnames to NAMES
    %output, but determine whether fields at final depth are structures or
    %not (g). After this, terminate loop by setting k to 0.
    if depth ~= -1                                                      ...
            && any(cellfun(@(x) size(find(x == '.'),2),NAMES) == depth)
        g = 1 ;
    elseif depth ~= -1                                                  ...
            && any(cellfun(@(x) size(find(x == '.'),2),NAMES) > depth)
        k = 0 ;
    end
end

%Return names of fields which are structures, as well as fields which are
%not structures, if 'full' optional argument is set.
if full == 1
    for i = 1:length(fndstruct)
        
        %If depth is specified, add structure names to appropriate depth to
        %output NAMES.
        if depth == -1 || size(find(fndstruct{i} == '.'),2) <= depth-1
            structi = find(cellfun(@(x) isempty(x) == 0                 ...
                ,strfind(NAMES,fndstruct{i})),1) ;
            
            %If the current structure name is related to the first field of
            %NAMES, add structure name to NAMES in a particular way, else
            %add it in another way.
            if structi > 1
                NAMES = cat(1,NAMES(1:(structi-1)),fndstruct{i}         ...
                    ,NAMES(structi:end)) ;
            elseif isempty(structi)
                error ('AATJ:fieldnamesr,NotFindStructure'              ...
                    ,'Could not find structure name in NAMES') ;
            else
                NAMES = cat(1,fndstruct{i},NAMES) ;
            end
        end
    end
end

%Return only fields which are structures, if optional argument is defined.
%If 'full' is not set, do not include both parent and child structures.
if struct == 1
    if full == 0
        fndstruct2 = {} ;
        for i = 1:length(fndstruct)
            if isempty(cell2mat(strfind(fndstruct,strcat(fndstruct{i},'.')))) == 1
                fndstruct2(end+1,1) = fndstruct(i) ;
            end 
        end
        fndstruct = fndstruct2 ;
    end
    NAMES = fndstruct ;
end

%Prefix input structure name on all fields of output cell array NAME if
%user set this option.
if prefix == 1
    structname = inputname(1) ;
    NAMES = cellfun(@(x) strcat(sprintf('%s.',structname),x(3:end))     ...
        ,NAMES,'UniformOutput',false) ;
else
    NAMES = cellfun(@(x) sprintf('%s',x(3:end)),NAMES,'UniformOutput'   ...
        ,false) ;
end
