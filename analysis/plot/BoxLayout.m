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

classdef BoxLayout < handle
  % BoxLayout - Gui layout manager
  %
  %   Abstract base class for Gui layout manager.  Used to dynamically
  %   size and position Gui elements.

  properties (Access = protected, Constant, Abstract)
    active_dim; % Dimension in which elements are appended to layout.
    free_dim;   % Dimension in which elements are free to expand in layout.
  end

  properties (Access = protected, Constant)
    text_extent_scaling = 1.15;   % Scale factor for extra room around string.
    title_extent_scaling = 1.15;  % Scale factor for extra room around title.
  end

  properties (Access = protected)
    item_list = {}; % List of LayoutElements contained in layout.
    container = []; % Gui container (e.g. uipanel) for layout.
    spacing = 1;    % Spacing in pixels between elements in layout.
    padding = 1;    % Padding in pixels around border of layout.
  end

  methods
    function obj = BoxLayout(varargin)
      % BoxLayout - Construct BoxLayout object.
      %
      %   Parameters:
      %     Spacing: Spacing in pixels between elements in layout.
      %     Padding: Padding in pixels around border of layout.

      p = inputParser();

      p.addParameter('Spacing', 1, @BoxLayout.ValidateNumericScalar);
      p.addParameter('Padding', 1, @BoxLayout.ValidateNumericScalar);
      p.parse(varargin{:});

      obj.spacing = p.Results.Spacing;
      obj.padding = p.Results.Padding;
    end


    function AddItem(obj, item, varargin)
      % AddItem - Add item to layout.
      %
      %   This method adds a Gui element or another BoxLayout to the layout.
      %   The order the elements are added is preserved in the layout.
      %
      %   Arguments:
      %     item: Gui element or BoxLayout to add to the layout.
      %
      %   Parameters:
      %     Stretch: Stretch factor for dynamically sizing element based
      %              on available space.  A value of <= 0 indicates no dynamic
      %              sizing.  A value of > 0 indicates a stretch factor
      %              relative to other dynamically sized elements in the same
      %              layout.
      %     MinSize: Custom minimum element size in pixels ([1x2]) to override
      %              the intrinsically derived value.  For dynamically sized
      %              elements this sets the minimum allowed size.  Otherwise,
      %              this sets the element size.

      p = inputParser();
      p.addParameter('Stretch', 0, @BoxLayout.ValidateNumericScalar);
      p.addParameter('MinSize', [0 0], @BoxLayout.ValidateNumeric1x2);
      p.parse(varargin{:});

      % Ensure item has bare necessities.
      assert(isa(item, 'BoxLayout') || ...
             (isprop(item, 'Position') && isprop(item, 'Parent')), ...
             'GUI item is missing Position or Parent property.');

      % Create LayoutElement from item and append to item_list.
      element = LayoutElement(item);
      element.Stretch = p.Results.Stretch;
      element.MinSize = p.Results.MinSize;
      element.CustomMinSize = ~ismember('MinSize', p.UsingDefaults);
      obj.item_list{end+1} = element;
    end


    function SetContainer(obj, container)
      % SetContainer - Sets Gui container for layout.
      %
      %   Arguments:
      %     container: Gui container (e.g. uipanel) to contain all elements
      %                within layout.  The Parent property of all immediate
      %                children of the layout will be set to this container.

      % Ensure container has bare necessities.
      assert(isprop(container, 'Parent'), ...
             'Container is missing Parent property.');
      obj.container = container;
    end


    function SetLayout(obj, fig)
      % SetLayout - Set layout manager of figure.
      %
      %   SetLayout should only be called on the root BoxLayout object once.
      %   This should occur after all gui elements and sub-layouts have been
      %   added.
      %
      %   Arguments:
      %     fig: Figure in which to layout elements contained in layout.

      assert(isgraphics(fig, 'figure'), 'Input not figure.');

      % Set all element's Parent property.
      obj.SetParents(fig);
      % Calculate minimum sizes of all elements and store root minimum size.
      min_size = obj.CalculateMinSizes();
      % Pass figure handle and root minimum size to resize callback.
      fig.SizeChangedFcn = @(~, ~) obj.ResizeCallback(fig, min_size);
    end
  end

  methods (Access = protected, Static = true)
    function ValidateNumericScalar(x)
      % ValidateNumericScalar - Validate numeric scalar input.

      assert(isnumeric(x) && numel(x) == 1, ...
             'Value must be scalar and numeric.');
    end


    function ValidateNumeric1x2(x)
      % ValidateNumeric1x2 - Validate numeric [1x2] array input.

      assert(isnumeric(x) && isequal(size(x), [1 2]), ...
             'Value must be a 1x2 numeric array.');
    end
  end

  methods (Access = protected)
    function ResizeCallback(obj, fig, min_size)
      % ResizeCallback - Callback for figure resize.

      % Create position for drawing and set lower bound on free dimension size.
      draw_pos = [0 0 fig.Position(3:4)];
      draw_pos(obj.free_dim + 2) = max(draw_pos(obj.free_dim + 2), ...
                                       min_size(obj.free_dim));
      obj.Draw(draw_pos);
    end


    function SetParents(obj, parent)
      % SetParents - Recursively set parents of elements contained in layout.

      % If layout has container, set container parent and set immediate
      % children's parent to said container.
      if ~isempty(obj.container)
        obj.container.Parent = parent;
        parent = obj.container;
      end

      for ii = 1:length(obj.item_list)
        item = obj.item_list{ii};
        if isa(item.Element, 'BoxLayout')
          item.Element.SetParents(parent);
        else
          item.Element.Parent = parent;
        end
      end
    end


    function min_size = CalculateMinSizes(obj)
      % CalculateMinSizes - Recursively calculate minimum element sizes.
      %
      %   CalculateMinSizes derives the minimum size of Gui elements based on
      %   the extent of their String property ([0 0] if non-existent) or the
      %   custom MinSize specified in AddItem.  Minimum sizes of layouts are
      %   derived by the maximum of the minimum sizes of layout children in the
      %   free dimension and by the sum of minimum sizes of layout children in
      %   the active dimension.  Layout minimum sizes include padding and
      %   spacing.  The minimum size of each element is stored in the
      %   corresponding LayoutElement object.
      %
      %  Return Values:
      %     min_size: minimum size in pixels ([width height]) of the layout.

      min_size = [0 0];

      % Empty layouts have zero minimum size.
      if isempty(obj.item_list)
        return;
      end

      for ii = 1:length(obj.item_list)
        item = obj.item_list{ii};
        if isa(item.Element, 'BoxLayout')
          item.MinSize = item.Element.CalculateMinSizes();
        else
          if ~item.CustomMinSize
            if isprop(item.Element, 'Extent')
              item.Element.Units = 'pixels';
              item.MinSize = (obj.text_extent_scaling * ...
                              item.Element.Extent(3:4));
            else
              item.MinSize = [0 0];
            end
          end
        end
        min_size(obj.active_dim) = (min_size(obj.active_dim) + ...
                                    item.MinSize(obj.active_dim));
        min_size(obj.free_dim) = max(min_size(obj.free_dim), ....
                                     item.MinSize(obj.free_dim));
      end

      % Account for spacing and padding.
      min_size(obj.active_dim) = (min_size(obj.active_dim) + ...
                                  (length(obj.item_list) - 1) * obj.spacing);
      min_size = min_size + 2 * obj.padding * [1 1];

      % Account for title and border of container.
      if ~isempty(obj.container)
        if ~isempty(obj.container.Title)
          obj.container.FontUnits = 'pixels';
          % TODO: Handle other title positions and remove size
          % fudging.
          min_size = (min_size + ...
                      [0 obj.title_extent_scaling * obj.container.FontSize]);
        end
        min_size = min_size + 2 * obj.container.BorderWidth * [1 1];
      end
    end


    function starting_pos = GetStartingPosition(obj, parent_position)
      % GetStartingPosition - Get starting position for layout draw.

      % Start at bottom left for HBoxLayout.
      if obj.active_dim == 1
        starting_pos = obj.padding * [1 1];
      % Start at top left for VBoxLayout.
      else
        starting_pos = obj.padding * [1 -1] + [0 parent_position(4)];
      end
      starting_pos = starting_pos + parent_position(1:2);
    end


    function next_pos = PrePositionElement(obj, cur_pos, element_size)
      % PrePositionElement - Update current position before element positioning.

      % No action for HBoxLayout.
      if obj.active_dim == 1
        next_pos = cur_pos;
      % Advance down for VBoxLayout.
      else
        next_pos = cur_pos + [0 -element_size(2)];
      end
    end


    function next_pos = PostPositionElement(obj, cur_pos, element_size)
      % PostPositionElement - Update current position after element positioning.

      % Advance right for HBoxLayout.
      if obj.active_dim == 1
        next_pos = cur_pos + [element_size(1) 0] + [obj.spacing 0];
      % Advance down for VBoxLayout.
      else
        next_pos = cur_pos + [0 -obj.spacing];
      end
    end


    function Draw(obj, parent_position)
      % Draw - Recursively draw layout elements within layout.
      %
      %   Arguments:
      %     parent_position: Bounding box in pixels ([x y width height]) within
      %                      which to draw Gui elements.

      % Gui element position is relative to containers.
      if ~isempty(obj.container)
        % Set container position to entirety of parent_position.
        obj.container.Units = 'pixels';
        obj.container.Position = parent_position;
        % Re-center parent_position relative to container and account for title
        % and border.
        parent_position(1:2) = [0 0];
        if ~isempty(obj.container.Title)
          obj.container.FontUnits = 'pixels';
          parent_position(4) = (parent_position(4) - ...
                                (obj.title_extent_scaling * ...
                                 obj.container.FontSize));
        end
        parent_position(3:4) = (parent_position(3:4) - ...
                                2 * obj.container.BorderWidth * [1 1]);
      end

      % Calculate remaining size in the active dimension available to stretch
      % elements by summing non-stretch element's minimum size and accounting
      % for spacing and padding.  Also, sum stretch values in order to calculate
      % stretch ratio.
      stretch_sum = 0;
      remaining_size = parent_position(obj.active_dim + 2);
      for ii = 1:length(obj.item_list)
        item = obj.item_list{ii};
        if item.Stretch > 0
          stretch_sum = stretch_sum + item.Stretch;
        else
          remaining_size = remaining_size - item.MinSize(obj.active_dim);
        end
      end
      remaining_size = (remaining_size - ...
                        (length(obj.item_list) - 1) * obj.spacing - ...
                        2 * obj.padding);

      % Get starting position for draw and initialize free dimension size.
      cur_pos = obj.GetStartingPosition(parent_position);
      element_size = [0 0];
      element_size(obj.free_dim) = (parent_position(obj.free_dim + 2) - ...
                                    2 * obj.padding);

      % Position each element, stretching where necessary.
      for ii = 1:length(obj.item_list)
        item = obj.item_list{ii};
        if item.Stretch > 0
          stretch_ratio = item.Stretch / stretch_sum;
          element_size(obj.active_dim) = max(stretch_ratio * remaining_size, ...
                                             item.MinSize(obj.active_dim));
        else
          element_size(obj.active_dim) = item.MinSize(obj.active_dim);
        end
        cur_pos = obj.PrePositionElement(cur_pos, element_size);
        if isa(item.Element, 'BoxLayout')
          item.Element.Draw([cur_pos element_size]);
        else
          item.Element.Units = 'pixels';
          item.Element.Position = [cur_pos element_size];
        end
        cur_pos = obj.PostPositionElement(cur_pos, element_size);
      end
    end
  end

end
