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
    collisionArray = helperClassManipCollisionsFromVisuals.createCollisionArray(robot);
end

%% Create robot cell
persistent worldCollisionArray
if isempty(worldCollisionArray)
    worldCollisionArray = {};
end

%% Check for collisions (exhaustive)
assert(length(collisionArray) > 1);
[isCollision,selfCollisionPairIdx,worldCollisionPairIdx] = ...
    checkCollisions(robot,collisionArray,worldCollisionArray,current_config,true);

%% Visualize and highlight bodies in collision
if interactive
    if ~isempty(worldCollisionArray)
        ax = visualizeCollisionEnvironment(worldCollisionArray);
        show(robot,current_config,"Parent",ax,"PreservePlot",false);
    else
        ax = show(robot,current_config);
    end

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
