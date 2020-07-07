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

function H5Plotter()
% H5Plotter - Interface for interactively plotting H5 data.

aio_nodes = [];
data_map = containers.Map();
gui = [];
log_full_path = '';
plot_axes = [];
basepath = fullfile(getenv('MAKANI_HOME'),'logs');
zero_time = 0.0;

SetupUi();

  function SetupUi()
    % SetupUi - Initialize and layout Gui element.

    gui.f = figure('Visible', 'off', 'Position', [100, 100, 350, 600], ...
                   'MenuBar', 'none', 'NumberTitle', 'off', ...
                   'Resize', 'on', 'Name', 'H5 Plotter');

    % Log File Panel
    gui.log_hbox = HBoxLayout('Padding', 4, 'Spacing', 3);
    gui.log_panel = uipanel('Title', 'H5 Log File');
    gui.log_hbox.SetContainer(gui.log_panel);
    gui.log_file_text = uicontrol('Style', 'edit', 'Enable', 'off', ...
                                  'String', ' ', 'HorizontalAlignment', 'left');
    gui.log_hbox.AddItem(gui.log_file_text, 'Stretch', 1);
    gui.log_choose_btn = uicontrol('Style', 'pushbutton', ...
                                   'String', 'Choose...', ...
                                   'Callback', @LogChooseCallback);
    gui.log_hbox.AddItem(gui.log_choose_btn);
    gui.log_load_last_btn = uicontrol('Style', 'pushbutton', ...
                                      'String', 'Load Last', ...
                                      'Callback', @LogLoadLastCallback);
    gui.log_hbox.AddItem(gui.log_load_last_btn);

    % Aio Node Panel.
    gui.aio_node_vbox = VBoxLayout('Padding', 4);
    gui.aio_node_panel = uipanel('Title', 'AIO Nodes');
    gui.aio_node_vbox.SetContainer(gui.aio_node_panel);
    gui.aio_node_list = uicontrol('Style', 'listbox', 'Max', 255, ...
                                  'Callback', @AioNodeListCallback);
    gui.aio_node_vbox.AddItem(gui.aio_node_list, 'Stretch', 1);

    % Aio Message Panel.
    gui.aio_msg_vbox = VBoxLayout('Padding', 4);
    gui.aio_msg_panel = uipanel('Title', 'AIO Messages');
    gui.aio_msg_vbox.SetContainer(gui.aio_msg_panel);
    % Semi-documented uitreenode and uitree require 'v0' first argument.
    % http://undocumentedmatlab.com/blog/uitree
    gui.aio_msg_tree_root = uitreenode('v0', 'AIO Messages', 'AIO Messages', ...
                                       [], false);
    [gui.aio_msg_tree, gui.aio_msg_container] = uitree('v0', ...
                                                       'Root', ...
                                                       gui.aio_msg_tree_root);
    gui.aio_msg_tree.setMultipleSelectionEnabled(true);
    gui.aio_msg_vbox.AddItem(gui.aio_msg_container, 'Stretch', 1);
    gui.plot_hbox = HBoxLayout();
    gui.aio_msg_vbox.AddItem(gui.plot_hbox);
    gui.plot_hbox.AddItem(HBoxLayout(), 'Stretch', 1);
    gui.aio_msg_plot_btn = uicontrol('Style', 'pushbutton', ...
                                     'String', 'Plot', ...
                                     'Callback', @AioMsgPlotCallback);
    gui.plot_hbox.AddItem(gui.aio_msg_plot_btn);

    % Aio Message Right-Click Context Menu.
    gui.export_menuitem = javax.swing.JMenuItem('Export to workspace.');
    set(gui.export_menuitem, 'ActionPerformedCallback', @ExportDataCallback);
    gui.context_jmenu = javax.swing.JPopupMenu();
    gui.context_jmenu.add(gui.export_menuitem);
    set(gui.aio_msg_tree.getTree(), 'MousePressedCallback', @ContextCallback);

    % Root layout.
    gui.root_vbox = VBoxLayout('Padding', 5);
    gui.root_vbox.AddItem(gui.log_hbox);
    gui.root_vbox.AddItem(gui.aio_node_vbox, 'Stretch', 1);
    gui.root_vbox.AddItem(gui.aio_msg_vbox, 'Stretch', 3);

    gui.root_vbox.SetLayout(gui.f);

    gui.f.HandleVisibility = 'callback';
    gui.f.Visible = 'on';
  end


  function LogChooseCallback(~, ~)
    % LogChooseCallback - Log choose button callback.

    [filename, newbasepath] = uigetfile(fullfile(basepath,'*.h5'));

    % Catch file selection failure.
    if filename == 0
      return;
    end

    % Only update path if successful file selection.
    basepath = newbasepath;
    OpenH5Log(basepath, filename);
  end


  function LogLoadLastCallback(~, ~)
    % LogLoadLastCallback - Log load last button callback.

    % Make sure a valid directory is specified.
    if ~exist(basepath, 'dir')
      return;
    end

    % Find most recent H5 log.
    log_files = dir(basepath);
    log_files = regexp({log_files.name}, '^[0-9]{8}-[0-9]{6}.*\.h5', 'match');
    log_files = sort([log_files{:}]);
    % Return if no log files can be found.
    if isempty(log_files)
      return;
    end

    OpenH5Log(basepath, log_files{end});
  end


  function OpenH5Log(path, filename)
    % OpenH5Log - Inspect H5 log and update Gui to reflect contents.
    %
    %   Arguments:
    %     path: Path to H5 log.
    %     filename: Filename of H5 log.

    full_path = fullfile(path, filename);
    log_info = GetH5LogInfo(h5info(full_path));

    % Catch invalid H5 log.
    if isempty(log_info)
      return;
    end

    % Save H5 log pathname.
    log_full_path = full_path;

    % Reset data containter and axes.
    data_map = containers.Map();
    plot_axes = [];

    % Reset time
    zero_time = 0.0;

    % Retrieve Aio nodes on log.
    aio_nodes = GetAioNodes(log_info);

    % Update GUI.
    gui.log_file_text.String = filename;
    if ~isempty(aio_nodes)
      % Set to empty array to avoid invalid index.
      gui.aio_node_list.Value = [];
      gui.aio_node_list.String = {aio_nodes.short_name};
      gui.aio_node_list.Value = 1;
    else
      gui.aio_node_list.Value = [];
      gui.aio_node_list.String = {};
    end
    % Call to update Aio message tree.
    AioNodeListCallback();
  end


  function info = GetH5LogInfo(info_struct)
    % GetH5LogInfo - Retrieve Aio message info from H5 log.
    %
    %   Arguments:
    %     info_struct: H5 info structure. Output of h5info.
    %
    %   Return Values:
    %     info: H5 info structure from "/messages" group.  Can be empty.

    info = [];
    for k = 1:length(info_struct.Groups)
      if strcmp(info_struct.Groups(k).Name, '/messages')
        info = info_struct.Groups(k);
        break;
      end
    end
  end


  function nodes = GetAioNodes(info)
    % GetAioNodes - Retrieve Aio nodes present in H5 log.
    %
    %   Arguments:
    %     info: H5 info structure from "/messages" group.
    %
    %   Return Values:
    %     nodes: Structure array of nodes.

    nodes = [];
    for k = 1:length(info.Groups)
      if ~isempty(info.Groups(k).Datasets)
        node = info.Groups(k);
        short_name = regexp(node.Name, '/messages/kAioNode(\w+)', ...
                            'tokens', 'once');
        if isempty(short_name)
          continue;
        end
        node.short_name = short_name{1};
        nodes = [nodes, node];  %#ok
      end
    end
  end


  function AioNodeListCallback(~, ~)
    % AioNodeListCallback - Handle an Aio node selection change.

    messages = GetCommonMessageTypes(aio_nodes(gui.aio_node_list.Value));
    PopulateMessageNodes(messages);
  end


  function messages = GetCommonMessageTypes(nodes)
    % GetCommonMessageTypes - Get common message types of selected Aio nodes.
    %
    %   Arguments:
    %     nodes: Structure array of H5 groups that represent Aio nodes.
    %
    %   Return Values:
    %     messages: Structure array of H5 datasets that represent Aio messages.

    messages = [];
    if isempty(nodes)
      return;
    end

    % Use first node as starting point.
    messages = nodes(1).Datasets;

    % Compare to all other nodes.
    for k = 2:length(nodes)
      % Find intersection between remaining names and current
      % node message names.
      [~, ind, ~] = intersect({messages.Name}, {nodes(k).Datasets.Name});
      % Only keep intersection.
      messages = messages(ind);
    end

    % Populate message short names.
    for k = 1:length(messages)
      short_name = regexp(messages(k).Name, 'kMessageType(\w+)', ...
                          'tokens', 'once');
      % If no valid short name, remove.
      if isempty(short_name)
        messages(k) = [];
      else
        messages(k).short_name = short_name{1};
      end
    end
  end


  function PopulateMessageNodes(messages)
    % PopulateMessageNodes - Populate the Aio message tree.
    %
    %   Arguments:
    %     messages: Structure array of H5 datasets that represent Aio messages.

    % Remove existing message types.
    gui.aio_msg_tree_root.removeAllChildren();

    for k = 1:length(messages)
      % Create tree node for each message type.
      node = uitreenode('v0', messages(k).short_name, ...
                        messages(k).short_name, [], false);
      gui.aio_msg_tree_root.add(node);

      % Add user data to node in order to track H5 dataset path and struct
      % member path.
      data.dataset_path = strcat('/', messages(k).Name);
      data.member_path = {};
      node.UserData = data;

      % Recursively fill members of message dataset.
      RecursivelyAddMemberNodes(node, messages(k).FillValue);
    end

    % Reload tree to show recently added tree nodes.
    gui.aio_msg_tree.reloadNode(gui.aio_msg_tree_root);
  end


  function RecursivelyAddMemberNodes(parent_node, struct)
    % RecursivelyAddMemberNodes - Recursively add tree nodes to parent_node.
    %
    %   Traverses the structure field names to populate parent_node. The
    %   parent_node's dataset and struct member paths are propagated through
    %   the node's user data.
    %
    %   Arguments:
    %     parent_node: Parent uitreenode to populate from struct.
    %     struct: H5 info structure representing members of compound dataset.

    field_names = fieldnames(struct);
    for k = 1:length(field_names)
      % If field is not a struct, the node's leaf attribute is set to true.
      node = uitreenode('v0', field_names{k}, field_names{k}, [], ...
                        ~isstruct(struct.(field_names{k})));

      % Append node's struct member name to parent's struct member path.
      data.dataset_path = parent_node.handle.UserData.dataset_path;
      data.member_path = [parent_node.UserData.member_path, field_names(k)];
      node.UserData = data;
      parent_node.add(node);

      % Continue if field is not a struct.
      if isstruct(struct.(field_names{k}))
        RecursivelyAddMemberNodes(node, struct.(field_names{k}));
      end
    end
  end


  function LoadData(aio_nodes, data_paths)
    % LoadData - Load H5 data for requested data paths.
    %
    %   Loads data for requested data paths and AIO nodes into global data_map.
    %   Only loads data that has not previously been loaded.
    %
    %   Arguments:
    %     aio_nodes: AIO nodes for which to load data.
    %     data_paths: Paths generated from GetSelectedNodePaths().

    for k = 1:length(data_paths)
      for m = 1:length(aio_nodes)
        dataset_full_path = strcat(aio_nodes(m).Name, ...
                                   data_paths(k).dataset);
        % Check to see if the H5 dataset that contains the desired member has
        % already been loaded.  If not, load data set and store in data_map.
        if ~isKey(data_map, dataset_full_path)
          % Specific messages have specialized utilities for loading data.
          % Otherwise, directly grab the data using h5read.
          if strfind(dataset_full_path, 'kMessageTypeMotorIsrDiag')
            [s.time, s.data.message] = ...
                LoadIsrDiag(log_full_path, dataset_full_path);
          else
            s.data = h5read(log_full_path, dataset_full_path);
            s.time = GetMessageTimes(s.data);
          end

          data_map(dataset_full_path) = s;
        end
      end
    end
  end


  function AioMsgPlotCallback(~, ~)
    % AioMsgPlotCallback - Plots H5 log data selected in the tree.

    % Check for selected Aio nodes.
    if isempty(aio_nodes)
      return;
    end
    if isempty(gui.aio_node_list.Value)
      return;
    end
    selected_aio_nodes = aio_nodes(gui.aio_node_list.Value);

    % Get the dataset and struct member paths to the selected nodes.
    paths = GetSelectedNodePaths();
    if isempty(paths)
      return;
    end

    % Indicate plotter is working.
    gui.f.Pointer = 'watch';
    drawnow();

    % Load desired data into data_map.
    LoadData(selected_aio_nodes, paths);

    % For each selected leaf node create a new plot.
    for k = 1:length(paths)
      label = GetDataLabel(paths(k));
      figure('Name', label, 'WindowStyle', 'docked', ...
             'NumberTitle', 'off');
      hold on;
      grid('on');

      % Collect axes for x axes linking later.
      plot_axes = [plot_axes, gca];  %#ok

      legend_entries = {};

      % For each selected Aio node add message data to shared plot.
      for m = 1:length(selected_aio_nodes)
        % Get dataset that contains the desired member and index into it
        % using the struct member path.
        dataset_full_path = strcat(selected_aio_nodes(m).Name, ...
                                   paths(k).dataset);
        msg = data_map(dataset_full_path);
        data = getfield(msg.data, paths(k).member{:});

        % Remove singleton dimensions if they exist.
        data_size = size(data);
        if length(data_size) > 2
          data = reshape(data, data_size(data_size ~= 1));
        end

        % The plot function doesn't know how to deal with arrays of chars.
        if ischar(data)
          data = uint8(data);
        end

        % Add legend based on size of data (single value or array).
        if size(data, 2) == 1
          legend_entries{end+1} = selected_aio_nodes(m).short_name;  %#ok
        else
          for n = 1:size(data, 1)
            short_name = selected_aio_nodes(m).short_name;
            legend_entries{end+1} = sprintf('%s[%d]', short_name, n - 1);  %#ok
          end
        end

        plot(msg.time, data);
      end

      % Create plot garnish using dataset and struct member paths for naming.
      title(label, 'Interpreter', 'none');
      ylabel(paths(k).member{end}, 'Interpreter', 'none');
      xlabel('Time [s]');
      [~, hObj] = legend(legend_entries, 'Interpreter', 'none');
      set(hObj, 'LineWidth', 0.75);
    end

    % Link all valid axes since last H5 log load.
    plot_axes = plot_axes(isvalid(plot_axes));
    linkaxes(plot_axes, 'x');

    % Gui is done working.
    gui.f.Pointer = 'arrow';
  end


  function paths = GetSelectedNodePaths()
    % GetSelectedNodePaths - Retrieves paths for selected Aio messages.
    %
    %   Return Values:
    %     paths: A structure array with members dataset and member that contains
    %            the paths for selected Aio message nodes.  Structure member
    %            dataset contains the H5 dataset path and structure member
    %            member contains a cell array of field names that specifies the
    %            path to the relevant data in the H5 compound dataset.

    % Retrieve selected nodes.
    dataset_nodes = gui.aio_msg_tree.getSelectedNodes();

    paths = [];
    for k = 1:length(dataset_nodes)
      node = dataset_nodes(k);
      % If node is leaf, append paths.
      if node.isLeaf()
        paths(end+1).dataset = node.handle.UserData.dataset_path;  %#ok
        paths(end).member = node.handle.UserData.member_path;
      % If not a leaf, enumerate through all children and append paths of
      % leaf nodes.
      else
        node_enumeration = node.breadthFirstEnumeration();
        while node_enumeration.hasMoreElements()
          node = node_enumeration.nextElement();
          if node.isLeaf()
            paths(end+1).dataset = node.handle.UserData.dataset_path;  %#ok
            paths(end).member = node.handle.UserData.member_path;
          end
        end
      end
    end
  end


  function label = GetDataLabel(path)
    % GetDataLabel - Create human-readable label Aio message path.
    %
    %   Arguments:
    %     path: A structure with members dataset and member containing the
    %           the H5 dataset and member path respectively.
    %
    %   Return Values:
    %     label: Human-readable string created from H5 dataset and member paths.

    msg_type = regexp(path.dataset, '/kMessageType(\w+)', 'tokens', 'once');
    if isempty(msg_type)
      msg_type = {''};
    end
    member_string = strjoin(path.member, '.');
    label = strcat(msg_type{1}, '.', member_string);
  end


  function times = GetMessageTimes(message)
    % GetMessageTimes - Returns the times of the dataset in seconds.
    %
    %   Arguments:
    %     message: A structure representing an Aio message dataset.
    %
    %   Return Values:
    %     times: An array of times in seconds relative to the start of the
    %            dataset.

    secs = double(message.capture_header.tv_sec);
    usecs = double(message.capture_header.tv_usec);

    % Use the same sense of zero time for all plots from same file.
    if zero_time == 0.0
      zero_time = secs(1) + 1e-6 * usecs(1);
    end

    times = secs + 1e-6 * usecs - zero_time;
  end


  function selected = IsNodeSelected(node, tree)
    % IsNodeSelected - Checks whether node is selected in tree.

    selected = false;
    selected_nodes = tree.getSelectedNodes();
    for k = 1:length(selected_nodes)
      if node == selected_nodes(k)
        selected = true;
        return;
      end
    end
  end


  function ContextCallback(~, event_data)
    % ContextCallback - Called on mouse press in AIO message tree.

    % Check if right click.
    if event_data.isMetaDown()
      click_x = event_data.getX();
      click_y = event_data.getY();
      jtree = gui.aio_msg_tree.getTree();
      tree_path = jtree.getPathForLocation(click_x, click_y);

      % Check if on node.
      if ~isempty(tree_path)
        node = tree_path.getLastPathComponent();

        % Check if node is not already selected and select.
        if ~IsNodeSelected(node, gui.aio_msg_tree)
          gui.aio_msg_tree.setSelectedNode(node);
        end

        % Show context menu.
        gui.context_jmenu.show(jtree, click_x, click_y);
        gui.context_jmenu.repaint;
      end
    end
  end

  function ExportDataCallback(~, ~)
    % ExportDataCallback - Export selected data to base workspace.

    % Check for selected Aio nodes.
    if isempty(aio_nodes)
      return;
    end
    if isempty(gui.aio_node_list.Value)
      return;
    end
    selected_aio_nodes = aio_nodes(gui.aio_node_list.Value);

    % Get the dataset and struct member paths to the selected nodes.
    paths = GetSelectedNodePaths();
    if isempty(paths)
      return;
    end

    % Load desired data into data_map.
    LoadData(selected_aio_nodes, paths);

    for k = 1:length(paths)
      for m = 1:length(selected_aio_nodes)
        % Get dataset that contains the desired member and index into it
        % using the struct member path.
        dataset_full_path = strcat(selected_aio_nodes(m).Name, ...
                                   paths(k).dataset);
        msg = data_map(dataset_full_path);

        % Create structure with data and time.
        data.val = getfield(msg.data, paths(k).member{:});
        data.time = msg.time;
        % Export structure to base workspace with AIO node prepended.
        assignin('base', strcat(selected_aio_nodes(m).short_name, '_', ...
                 paths(k).member{end}), data);
      end
    end
  end
end
