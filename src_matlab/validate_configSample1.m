% Sample script to demonstrate execution of function [isCollision, selfCollisionPairIdx, worldCollisionPairIdx] = validate_config(current_config, urdf_path)
current_config = [0, 0, pi/2, 0, pi/2, 0]'; % Initialize current_config here
urdf_path = 'D:\Nutzer\Documents\PycharmProjects\gcode\ressource\robot.urdf'; % Initialize urdf_path here
[isCollision, selfCollisionPairIdx, worldCollisionPairIdx] = validate_config(current_config, urdf_path);
