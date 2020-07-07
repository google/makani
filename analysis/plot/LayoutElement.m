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

classdef LayoutElement < handle
  % LayoutElement - Layout manager element.

  properties
    Element;                % Gui element or layout manager handle.
    Stretch = 0;            % Stretch value of layout element.
    MinSize = [0 0];        % Minimum size in pixels ([w h]) of element.
    CustomMinSize = false;  % Boolean defining whether MinSize is intrinsic.
  end

  methods
    function obj = LayoutElement(element)
      % LayoutElement - Construct LayoutElement object.
      %
      %   Arguments:
      %     element: Gui element or layout manager handle .

      obj.Element = element;
    end
  end

end
