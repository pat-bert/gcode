function [isColl, selfCollPairIdx, worldCollPairIdx, extrusionPairIdx] = validate_config(config, scene_idx, interactive, urdf, extr_vertices)
%VALIDATE_CONFIG Check whether a robot configuration is collision free.
%   isColl = validate_config(config, scene_idx, interactive, urdf)
%   Check a robot model for collisions with itself, its cell and extruded
%   filament. 
%   config      - Configuration given as column vector of joint coordinates
%   interactive - Flag to specify whether a plot should be created
%   urdf        - String to specifiy the file path for the URDF file
%                 containing the model of the robot and its cell
%   extr_vertices - Cell array of nx3 matrices representing the vertices of
%                   extruded objects
%   scene_idx     - An index to specify up to which extrusion object
%                   collision checking will be done.
%
%   The filepath for the URDF can be omitted on repeated calls. The robot
%   model and all collision meshes will be used from the previous call. The
%   robot configuration and the scene index can be changed to check a
%   different configuration in a different state of the robot environment.
%
%   [isColl, selfCollPairIdx, worldCollPairIdx, extrusionPairIdx] 
%   = validate_config(config, scene_idx, interactive, urdf, extr_vertices)
%
%   Additional information is returned:
%   selfCollPairIdx     - Index pairs of the arm elements that are in
%                         collision
%   worldCollPairIdx    - Index pairs of the arm element and the part of
%                         the robot cell that are in collision
%   extrusionPairIdx    - First index pair of the arm element and the 
%                         extrusion object that are in collision

%% Declare static variables
% Robot Tree
persistent robot
% Collision object for robot base
persistent base_obj
% Collision objects for robot arm elements without base
persistent robot_arm_objects
% Collision objects for robot cell elements
persistent robot_cell_objects
% Collision objects for extruded filament
persistent extrusion_objects
persistent arm_element_names

%% Initialize outputs
isColl = false;
selfCollPairIdx = [];
worldCollPairIdx = [];
extrusionPairIdx = [];

%% Calculate persistent variables
if isempty(robot) || (exist('urdf', 'var') && ~isempty(urdf))
    if ~exist('urdf', 'var') || isempty(urdf)
        % Invalid because URDF was never given
        isColl = -1;
        selfCollPairIdx = -1;
        worldCollPairIdx = -1;
        return;
    end
    %% Create robot tree structure from URDF
    % 'D:\Nutzer\Documents\PycharmProjects\gcode\ressource\robot.urdf'
    robot = importrobot(urdf);
    
    %% Figure out base
    try
        base = robot.getBody('base');
    catch
        disp('Base was not found!')
        isColl = -1;
        selfCollPairIdx = -1;
        worldCollPairIdx = -1;
        return;
    end
    
    % Keep track of bodies already used
    identified_body_names = cell(numel(robot.Bodies) + 1);
    identified_count = 1;
    identified_body_names{identified_count} = base.Name;
    identified_count = identified_count + 1;
    
    % Create collision object for base
    base_obj = helperClassManipCollisionsFromVisuals.createCollisionObj(base);
    
    %% Figure out robot arm elements as children of base
    parent = base;
    robot_arm = cell(1, 2);
    idx = 1;
    while(numel(parent.Children) > 0)
        % Select next link
        parent = parent.Children{1};
        identified_body_names{identified_count} = parent.Name;
        identified_count = identified_count + 1;
        % Create collision object for current link
        robot_arm(idx, :) =  helperClassManipCollisionsFromVisuals.createCollisionObj(parent);
        idx = idx + 1;
    end
    
    % Assign link collision objects to static variable
    robot_arm_objects = robot_arm;
    
    %% Figure out robot cell bodies
    cell_bodies = cell(1,2);
    idx = 1;
    
    identified_body_names = identified_body_names(~cellfun('isempty', identified_body_names));
    
    % Add base if it was not used before
    if ~any(strcmp(robot.Base.Name, identified_body_names))
        cell_bodies(idx, :) = helperClassManipCollisionsFromVisuals.createCollisionObj(robot.Base);
        idx = idx + 1;
    end
    
    % Add arm elements that were not used before
    for i=1:numel(robot.Bodies)
        if ~any(strcmp(robot.Bodies{i}.Name, identified_body_names))
            cell_bodies(idx, :) = helperClassManipCollisionsFromVisuals.createCollisionObj(robot.Bodies{i});
            idx = idx + 1;
        end
    end
    
    % Assign robot cell collision objects to static variable
    robot_cell_objects = cell_bodies;
    arm_element_names = identified_body_names;
end

%% Create filament collision meshes
if exist('extr_vertices', 'var') && ~isempty(extr_vertices)
    % Allocate cell array
    extrusion_obj = cell(length(extr_vertices),1);
    for i=1:length(extr_vertices)
        if ~isempty(extr_vertices{i})
            extrusion_obj{i} = collisionMesh(extr_vertices{i});
        else
            % Set empty object to be skipped later on
            % This allows common handling of paths with and without
            % extrusion
            extrusion_obj{i} = [];
        end
    end
    extrusion_objects = extrusion_obj;
end

%% Update transforms
% Rather than calling getTransform at each loop, populate a transform
% tree, which is a cell array of all body transforms with respect to
% the base frame
transformTree = cell(length(robot_arm_objects) + 1,1);

% Initialize key parameters
robot.DataFormat = 'column';

% For the base, this is the identity
transformTree{1} = eye(4);
% Remaining arm elements
for i = 2:numel(arm_element_names)
    transformTree{i} = getTransform(robot, config, arm_element_names{i});
end

%% Update poses of arm collision objects
for i = 1:length(robot_arm_objects)
    robot_arm_objects{i,1}.Pose = transformTree{i+1} * robot_arm_objects{i,2};
end

%% Set poses of robot cell objects
for i = 1:length(robot_cell_objects)
    robot_cell_objects{i,1}.Pose = robot_cell_objects{i,2};
end

%% Collisions of arm elements
for arm_idx=1:length(robot_arm_objects)
    % Check whether arm has a collision object at all
    if ~isempty(robot_arm_objects{arm_idx,1})
        % Check collisions with base if element is more than one apart
        if arm_idx > 1
            % Check whether base has a collision object
            if ~isempty(base_obj{1,1})
                % Check for local collision and update the overall
                % collision status flag
                local_coll = checkCollision(robot_arm_objects{arm_idx}, base_obj{1});
                isColl = isColl || local_coll;
                
                % Add index for self collision
                if local_coll
                    selfCollPairIdx = [selfCollPairIdx; [1 arm_idx + 1]]; %#ok<AGROW>
                end
            end
        end
        
        % Check collisions with other arm elements
        for other_arm_idx=arm_idx:length(robot_arm_objects)
            % Do not check collisions with self or neighbors (4 is concave
            % so always in collision with 6)
            if (other_arm_idx ~= arm_idx) && ...
                    (other_arm_idx ~= arm_idx + 1) && ...
                    (other_arm_idx ~= arm_idx - 1) && ...
                    ~(arm_idx == 4 && other_arm_idx == 6)
                % Check whether other arm has a collision object
                if ~isempty(robot_arm_objects{other_arm_idx,1})
                    % Check for local collision and update the overall
                    % collision status flag
                    local_coll = checkCollision(robot_arm_objects{arm_idx}, robot_arm_objects{other_arm_idx});
                    isColl = isColl || local_coll;
                    
                    % Add index for self collision
                    if local_coll
                        selfCollPairIdx = [selfCollPairIdx; [arm_idx + 1 other_arm_idx + 1]]; %#ok<AGROW>
                    end
                end
            end
        end
        
        % Check collision with each object of the robot cell
        for cell_idx=1:length(robot_cell_objects)
            % Check whether robot cell object has a collision object
            if ~isempty(robot_cell_objects{cell_idx,1})
                % Check for local collision and update the overall
                % collision status flag
                local_coll = checkCollision(robot_arm_objects{arm_idx}, robot_cell_objects{cell_idx});
                isColl = isColl || local_coll;
                
                % Add index for cell collision
                if local_coll
                    worldCollPairIdx = [worldCollPairIdx; [arm_idx + 1 cell_idx]]; %#ok<AGROW>
                end
            end
        end
    end
end

%% Collisions of arm elements with extruded objects
% Skip very long collision checking if the pose is already invalid
if ~isColl && ~isempty(extrusion_objects)
    if ~exist('scene_idx', 'var') || isempty(scene_idx)
        scene_idx = Inf;
    end
    
    % Iterate over each extrusion object until the specified scene index
    for L=1:min(length(extrusion_objects), scene_idx)
        % Check collisions with each arm element (base cannot collide)
        for arm_idx=1:length(robot_arm_objects)
            % Check whether a collision mesh is available
            if ~isempty(robot_arm_objects{arm_idx,1}) && ~isempty(extrusion_objects{L})
                % Check for local collision and update the overall
                % collision status flag
                local_coll = checkCollision(robot_arm_objects{arm_idx}, extrusion_objects{L});
                isColl = isColl || local_coll;
                
                % Set indices
                if local_coll
                    extrusionPairIdx = [extrusionPairIdx; [arm_idx + 1 L]]; %#ok<AGROW>
                end
            end
        end
    end
end

%% Visualize and highlight bodies in collision if collision was detected
if interactive && isColl
    figure()
    
    %% Show all the existing objects up to the index
    if ~isempty(robot_cell_objects)
        ax = visualizeCollisionEnvironment(robot_cell_objects(:,1));
        show(robot,config,"Parent",ax,"PreservePlot",false);
    else
        ax = show(robot,config);
    end
    if ~isempty(extrusion_objects)
        existing_extrusions = extrusion_objects(1:min(length(extrusion_objects), scene_idx));
        existing_extrusions = existing_extrusions(~cellfun('isempty', existing_extrusions));
        for i=1:numel(existing_extrusions)
            show(existing_extrusions{i})
        end
    end
    
    selfCollPairIdx = double(selfCollPairIdx);
    worldCollPairIdx = double(worldCollPairIdx);
    extrusionPairIdx = double(extrusionPairIdx);

    %% Mark the collision objects in a different color
    problemBodies = [];
    if ~isempty(worldCollPairIdx)
        problemBodies = [problemBodies worldCollPairIdx*[1 0]'];
    end
    if ~isempty(selfCollPairIdx)
        selfCollPairIdx = reshape(selfCollPairIdx, [], 1);
        problemBodies = [problemBodies; unique(selfCollPairIdx)];
    end
    if ~isempty(extrusionPairIdx)
        problemBodies = [problemBodies extrusionPairIdx*[1 0]'];
    end
    if ~isempty(problemBodies)
        problemBodies = unique(problemBodies);
        % Get subtree from base
        base = robot.getBody('base');
        robot_arm_tree = subtree(robot, base.Children{1}.Name);
        exampleHelperHighlightCollisionBodies(robot_arm_tree,problemBodies,ax);
    end
end
end
