# Functional Redundancy Resolution
from typing import List

import numpy as np

from src.kinematics.forward_kinematics import geometric_jacobian, right_generalized_inverse_jacobian
from src.kinematics.joints import BaseJoint


def frr(config: List[BaseJoint], joints: List[float], tool_vector: List[float], h: List[float]):
    # Calculate jacobian and its righ-hand pseudo-inverse
    jac = geometric_jacobian(config, joints)
    pinv_jac = right_generalized_inverse_jacobian(jac)

    # Calculate tool vector, its transposed version and the projector
    e = np.asarray(tool_vector)
    e_t = e.transpose()
    t_proj = e @ e_t

    # Transform the manipulation vector and calculate the delta joints
    h = np.asarray(h)
    delta_joints = pinv_jac @ np.array([np.zeros(3, 1), t_proj @ jac[3:, :] @ h])

    return joints + list(delta_joints)
