function [isInCollision, selfCollisionPairIdx, worldCollisionPairIdx] = checkCollisions(tree, bodyCollisionArray, worldCollisionArray, config, isExhaustiveChecking)
% This function is for internal use only and may be removed in a future release

%exampleHelperManipCheckSelfCollisions Check for collisions between robot bodies
%   This example accepts a rigidBodyTree object, TREE, and COLLISIONARRAY,
%   a cell array of collision meshes and the relationship of their origin
%   to the robot. For a given robot configuration, CONFIG, the function
%   checks whether the robot is in collision with itself. If
%   ISEXHAUSTIVECHECKING is set to FALSE, the function will exit as soon as
%   a collision is found. The elements of COLLISIONARRAY are ordered so
%   that the their indices correspond to the indices of the rigid bodies in
%   the vector [tree.Base tree.Bodies], where the first row corresponds to
%   the base of the rigidBodyTree, the second element corresponds to
%   tree.Bodies{1}, the third to tree.Bodies{2}, etc. The function outputs
%   a flag indicating whether collision is detected, ISINCOLLISION, and an
%   Mx2 matrix of collision body indices that indicate which objects in the
%   COLLISIONARRAY are in collision with each other.

%   Copyright 2019 The MathWorks, Inc.

    % Basic validation
    narginchk(5,5)
    validateattributes(tree, {'robotics.RigidBodyTree'}, {'nonempty'}, 'exampleHelperManipSelfCollisions', 'tree');
    validateattributes(bodyCollisionArray, {'cell'}, {'nonempty','nrows',tree.NumBodies+1, 'ncols', 2}, 'exampleHelperManipSelfCollisions', 'tree');
    validateattributes(config, {'double'}, {'nonempty','vector','nrows',length(tree.homeConfiguration)}, 'exampleHelperManipSelfCollisions', 'config');
    validateattributes(isExhaustiveChecking, {'logical'}, {'nonempty','scalar'}, 'exampleHelperManipSelfCollisions', 'isExhaustiveChecking');

    % Initialize the outputs
    isInCollision = false;
    selfCollisionPairIdx = [];
    worldCollisionPairIdx = [];
    
    % Initialize key parameters
    tree.DataFormat = 'column';
    robotBodies = [{tree.Base} tree.Bodies];
    
    % Rather than calling getTransform at each loop, populate a transform
    % tree, which is a cell array of all body transforms with respect to
    % the base frame
    transformTree = cell(numel(robotBodies),1); 
    
    % For the base, this is the identity
    transformTree{1} = eye(4);
    for i = 1:numel(robotBodies)
        transformTree{i} = getTransform(tree, config, robotBodies{i}.Name);
    end

    % Iterate over all bodies
    for j = 1:numel(robotBodies)
        % Since collisionPairIdx order doesn't matter, only check every
        % pair of bodies once
        for k = j:numel(robotBodies)
            % Don't check collision with self or neighbors
            if j ~= k && j ~= k+1 && j ~= k-1 && ~(j == 5 && k == 7)% not checking with self and neighbors, plus 5&&7 is false positive
                % Ensure that both bodies have associated collision objects
                if ~isempty(bodyCollisionArray{j,1}) && ~isempty(bodyCollisionArray{k,1})

                    % Get the collision object pose from the associated
                    % rigid body tree. The updated pose is the product of
                    % the associated rigid body tree pose and transform
                    % that relates the collision object origin to the rigid
                    % body position (measured from the parent joint).
                    bodyCollisionArray{j,1}.Pose = transformTree{j}*bodyCollisionArray{j,2};
                    bodyCollisionArray{k,1}.Pose = transformTree{k}*bodyCollisionArray{k,2};

                    % Check for local collision and update the overall
                    % collision status flag
                    localCollisionStatus = checkCollision(bodyCollisionArray{j}, bodyCollisionArray{k});
                    isInCollision = isInCollision || localCollisionStatus;

                    % If a collision is detected, update the matrix of bodies
                    % in collision
                    if localCollisionStatus
                        selfCollisionPairIdx = [selfCollisionPairIdx; [j k]]; %#ok<AGROW>
                        
                        if ~isExhaustiveChecking
                            return;
                        end
                    end
                end        
            end
        end
        
        % Check collisions with all the world collision objects
        if j > 1
            for m = 1:numel(worldCollisionArray)
                if ~isempty(bodyCollisionArray{j,1})
                    % The body poses have already been updated in the previous
                    % for-loop, and the world collision objects don't move, so
                    % there's no need to update poses

                    % Check for local collision and update the overall
                    % collision status flag
                    localCollisionStatus = checkCollision(bodyCollisionArray{j}, worldCollisionArray{m});
                    isInCollision = isInCollision || localCollisionStatus;

                    % If a collision is detected, update the matrix of bodies
                    % in collision
                    if localCollisionStatus
                        worldCollisionPairIdx = [worldCollisionPairIdx; [j m]]; %#ok<AGROW>

                        if ~isExhaustiveChecking
                            return;
                        end
                    end
                end
            end
        end
    end

end