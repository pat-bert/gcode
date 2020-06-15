from src.kinematics.joints import BaseJoint
from src.prechecks.configs import melfa_rv_4a


def test_melfa_rv_4a():
    config = melfa_rv_4a()

    assert len(config) == 6
    assert all([isinstance(jnt, BaseJoint) for jnt in config])
