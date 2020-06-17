class TrajectoryError(ValueError):
    """
    Indicate that a trajectory check has failed.
    """


class CollisionViolation(TrajectoryError):
    """

    """


class WorkspaceViolation(TrajectoryError):
    """
    Will be raised if no solution for the IK-problem can be found for a given pose.
    Cannot be recovered.
    """


class CartesianLimitViolation(TrajectoryError):
    """
    Will be raised if any position of the trajectory is outside of the specified cartesian cuboid.
    Cannot be recovered.
    """


class JointLimitViolation(TrajectoryError):
    """
    Will be raised if the positional limits of the joints are violated in each solution for a point.
    In that case the trajectory is outside of the reduced joint workspace.
    The reduced joint workspace is defined by the manufacturer's joint limits or by user-defined ones within.
    Cannot be recovered.
    """


class JointVelocityViolation(TrajectoryError):
    """
    Will be raised if the the joint velocity would be exceeded.
    In that case a singularity is present or the speed of the TCP has to be lowered.
    """


class ConfigurationChangesError(TrajectoryError):
    """
    Will be raised if a segment is not accessible within at least one common configuration.
    Since start and end points are included in segments also continuity between segments
    is tested.
    In case of a configuration change a singularity needs to be crossed and a path with the least
    configuration changes might be found.
    """
