function [isCollision, selfCollisionPairIdx, worldCollisionPairIdx] = validate_config(current_config, interactive, urdf_path)
persistent robot
persistent collisionArray
if isempty(robot) || nargin == 3
    if nargin < 3
       isCollision = -1;
       selfCollisionPairIdx = -1;
       worldCollisionPairIdx = -1;
       return;
    end
    %% Import robot
    % 'D:\Nutzer\Documents\PycharmProjects\gcode\ressource\robot.urdf'
    robot = importrobot(urdf_path);

    %% Set home position (not contained in URDF)
    home_config = [0 0 pi/2 0 pi/2 0];
    for i=1:length(home_config)
        robot.Bodies{i}.Joint.HomePosition = home_config(i);   
    end

    collisionArray = helperClassManipCollisionsFromVisuals.createCollisionArray(robot);
end

%% Robot Cell Parameters
wall_thickness = 50;
cell_side_length = 1000;
wall_offset = 500;
wall_height = 1000;

%% Create robot cell
persistent worldCollisionArray
if isempty(worldCollisionArray)
    floor_tf = [
        [eye(3), [0; 0; -wall_thickness/2]]
        [0, 0, 0, 1]
    ];
    left_wall_tf = [
        [eye(3), [0; -wall_offset - wall_thickness/2; +wall_height/2]]
        [0, 0, 0, 1]
    ]; 
    right_wall_tf = [
        [eye(3), [0; +wall_offset + wall_thickness/2; +wall_height/2]]
        [0, 0, 0, 1]
    ]; 

    floor = collisionBox(cell_side_length, cell_side_length, wall_thickness);
    floor.Pose = floor_tf;

    left_wall = collisionBox(cell_side_length, wall_thickness, wall_height);
    left_wall.Pose = left_wall_tf;

    right_wall = collisionBox(cell_side_length, wall_thickness, wall_height);
    right_wall.Pose = right_wall_tf;

    worldCollisionArray = {right_wall left_wall floor};
end

%% Check for collisions (exhaustive)
assert(length(collisionArray) > 1);
[isCollision,selfCollisionPairIdx,worldCollisionPairIdx] = ...
    checkCollisions(robot,collisionArray,worldCollisionArray,current_config,true);

%% Visualize and highlight bodies in collision
if interactive
    ax = visualizeCollisionEnvironment(worldCollisionArray);
    show(robot,current_config,"Parent",ax,"PreservePlot",false);

    problemBodies = [];
    if ~isempty(worldCollisionPairIdx)
        problemBodies = [problemBodies worldCollisionPairIdx*[1 0]'];
    end
    if ~isempty(selfCollisionPairIdx)
        selfCollisionPairIdx = reshape(selfCollisionPairIdx, [], 1);
        problemBodies = [problemBodies; unique(selfCollisionPairIdx)];
    end
    if ~isempty(problemBodies)   
        problemBodies = unique(problemBodies);
        exampleHelperHighlightCollisionBodies(robot,problemBodies,ax);
    end
end
end
