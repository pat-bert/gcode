#!/usr/bin/env python
"""
Sample script that uses the CollisionChecking module created using
MATLAB Compiler SDK.

Refer to the MATLAB Compiler SDK documentation for more information.
"""

from __future__ import print_function
import CollisionChecking
import matlab

my_CollisionChecking = CollisionChecking.initialize()

current_configIn = matlab.double([0.0, 0.0, 1.5707963267948966, 0.0, 1.5707963267948966, 0.0], size=(6, 1))
urdf_pathIn = "D:\\Nutzer\\Documents\\PycharmProjects\\gcode\\ressource\\robot.urdf"
isCollisionOut, selfCollisionPairIdxOut, worldCollisionPairIdxOut = my_CollisionChecking.validate_config(
    current_configIn, urdf_pathIn, nargout=3)
print(isCollisionOut, selfCollisionPairIdxOut, worldCollisionPairIdxOut, sep='\n')

my_CollisionChecking.terminate()
