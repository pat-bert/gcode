from math import sqrt
from typing import List


class BadSpeedParameter(ValueError):
    """
    Raised if a speed profile fails due to bad parameters.
    """


def trapezoidal_speed_profile(v_max: float, acc: float, s_total: float, ds: float) -> List[float]:
    """

    :param v_max: Peak velocity in mm/s
    :param acc: Constant acceleration in mm/s^2
    :param s_total: Total way in mm
    :param ds: Constant way sample in mm
    :return: List of time points given in seconds
    """
    if v_max > sqrt(s_total * acc):
        # print(f'Cannot accelerate by {acc} mm/s^2 to {v_max} mm/s within {s_total} mm.')
        v_max = round(sqrt(s_total * acc))
        # print(f'Reduced speed to {v_max} mm/s.')

    # Times
    acc = abs(acc)
    try:
        t_acc = v_max / acc
        t_total = t_acc + s_total / v_max
    except ZeroDivisionError as e:
        raise BadSpeedParameter('Cannot calculate speed profile for zero acceleration or zero velocity.') from e

    t_cruise = t_total - 2 * t_acc
    t = []

    # Distances
    s_acc_end = 0.5 * acc * t_acc ** 2
    s_cruise_end = s_acc_end + v_max * t_cruise
    s_curr = 0

    if ds > s_total:
        # We still need to give a time delta to calculate the velocities correctly
        return [0, t_total]

    while s_curr <= s_total:
        if s_curr <= s_acc_end:
            # Acceleration phase
            t.append(sqrt(2 * s_curr / acc))
        elif s_curr <= s_cruise_end:
            # Cruise phase
            t.append(t[-1] + ds / v_max)
        else:
            # Deceleration phase
            t.append(t_total - sqrt(2 * (s_total - s_curr) / acc))
        s_curr += ds

    return t
