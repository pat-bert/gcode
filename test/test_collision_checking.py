from math import pi

import pytest

from src.collisions.collision_checking import MatlabCollisionChecker


@pytest.mark.slow
def test_check_collisions():
    eng = MatlabCollisionChecker()
    valid_joints = [0, 0, pi / 2, 0, pi / 2, 0]
    invalid_joints = [0, 0, 0, 0, pi / 2, 0]

    with pytest.raises(ValueError):
        eng.check_collisions(valid_joints)

    is_collision, self_collision, world_collision = eng.check_collisions(valid_joints, path='./../ressource/robot.urdf')

    assert not is_collision
    assert len(self_collision) == 0
    assert len(world_collision) == 0

    is_collision, self_collision, world_collision = eng.check_collisions(invalid_joints)

    assert is_collision
    assert len(self_collision) > 0
