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

classdef VBoxLayout < BoxLayout
  % VBoxLayout - Gui vertical layout manager.
  %
  %   Use to dynamically size and position Gui elements in a vertical pattern.

  properties (Access = protected, Constant)
    active_dim = 2; % Dimension in which elements are appended to layout.
    free_dim = 1;   % Dimension in which elements are free to expand in layout.
  end

  methods
    function obj = VBoxLayout(varargin)
      % VBoxLayout - Construct VBoxLayout object.
      %
      % See also BoxLayout.
      obj = obj@BoxLayout(varargin{:});
    end
  end

end
