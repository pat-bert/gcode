classdef helperClassManipCollisionsFromVisuals < robotics.manip.internal.InternalAccess
    % This file is for internal use only and may be modified or removed in
    % a future release.
    %
    %helperClassManipCollisionsFromVisuals Create collision array from visuals
    
    %   Copyright 2019 The MathWorks, Inc.=
    
    %% Static methods
    
    methods (Static)
        function collisionArray = createCollisionArray(tree)
            %createCollisionArrayFromVisuals Create an array of collision objects from the visual meshes
            %   This function creates a cell array of bodies in the input
            %   rigidBodyTree object, TREE, that has the order
            %   treeBodies = [{tree.Base} tree.Bodies]. The method then uses the mesh
            %   visual information in each of those bodies to generate a
            %   corresponding cell array of collision meshes and
            %   transforms, COLLISIONARRAY. The first element of each row
            %   in COLLISIONARRAY is the collisionMesh that corresponds the
            %   visual of treeBody with the equivalent index. The second
            %   element is the transform that maps the origin of the
            %   collision mesh to the rigidBody origin.
            
            % Initialize array
            collisionArray = cell(tree.NumBodies+1, 2);
            
            % For each of the bodies, get the body internal, which links to
            % the visual information
            allBodies = [{tree.Base} tree.Bodies];
            for i = 1:numel(allBodies)
                if i == 1
                    bodyInternal = tree.Base.BodyInternal;
                else
                    bodyInternal = tree.Bodies{i-1}.BodyInternal;
                end
                
                % Use the mesh from the visuals internal. For simplicity,
                % assume that the first visual is always the one that gets
                % used if there is more than one visual.
                visualsInternal = bodyInternal.VisualsInternal;
                if ~isempty(visualsInternal)
                    collisionArray{i,1} = collisionMesh(double(visualsInternal{1}.Vertices));
                    collisionArray{i,2} = double(visualsInternal{1}.Tform);
                end
            end
        end
    end
    
end

