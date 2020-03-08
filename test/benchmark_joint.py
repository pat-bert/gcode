import timeit
from math import pi

if __name__ == '__main__':

    input_parameters = [
        # a | alpha | d | theta | actual
        # Translational joints
        (30, 2.7, None, 0, 10, 'Translational only'),
        # Rotational joints:
        (30, 2.7, -15, None, 2.7, 'General Rotational'),
        (0, 2.7, -15, None, 2.7, 'No Offset'),
        (30, pi / 2, -15, None, 2.7, 'Perpendicular'),
        (30, -pi / 2, -15, None, 2.7, 'Perpendicular'),
        (30, 0, -15, None, 2.7, 'No twist'),
        (30, pi, -15, None, 2.7, 'No twist'),
        (30, -pi, -15, None, 2.7, 'No twist'),
        (0, pi, -15, None, 2.7, 'No twist and no offset'),
        (0, pi / 2, -15, None, 2.7, 'Perpendicular and no offset'),
        (0, -pi / 2, -15, None, 2.7, 'Perpendicular and no offset'),
    ]

    repetitions = 100000

    for test in input_parameters:
        print("Testing <{}>".format(test[5]))
        # Time setup
        time = timeit.timeit(
            stmt='BaseJointFactory.new(a={a}, alpha={alpha}, d={d}, theta={theta})'.format(
                a=test[0], alpha=test[1], d=test[2], theta=test[3]),
            number=repetitions,
            setup='from src.kinematics.joint_factories import BaseJointFactory')
        print('Setup time: {} seconds'.format(time / repetitions))

        # Time multiplication
        time = timeit.timeit(
            stmt='joint.mul(theta=theta, d=d)',
            number=repetitions,
            setup="""
from src.kinematics.joint_factories import BaseJointFactory
joint = BaseJointFactory.new(a={a}, alpha={alpha}, d={d}, theta={theta})
d = {d}
theta = {theta}
if d is None:
    d = {actual_val}
if theta is None:
    theta = {actual_val}
            """.format(a=test[0], alpha=test[1], d=test[2], theta=test[3], actual_val=test[4])
        )
        print('Calc time: {} seconds'.format(time / repetitions))
