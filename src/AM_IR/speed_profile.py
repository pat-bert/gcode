from typing import List

import matplotlib.pyplot as plt
from scipy.integrate import simps


def calc_average(t: List[float], v: List[float]) -> float:
    sum_sample = simps(v, t, even='avg')
    return sum_sample / t[-1]


def draw_speed(target_velocity: float, time: List[float], velocity_samples: List[float]) -> None:
    """
    Plots a speed profile and visualizes the average
    :param target_velocity: Value for comparison
    :param time: List of time samples
    :param velocity_samples: List of velocity samples
    :return:
    """
    avg = calc_average(time, velocity_samples)

    ax: plt.Axes = plt.axes()
    ax.grid()
    ax.plot(time, velocity_samples, label='Velocity')
    ax.plot(time, [avg] * len(time), label='Average', linestyle='--')
    ax.plot(time, [target_velocity] * len(time), label='Target', linestyle='-.')

    ax.set(xlim=(0, 1.1 * time[-1]), ylim=(0, 1.1 * max(velocity_samples)), xlabel='Time in s',
           ylabel='Velocity in mm/s')
    plt.title("Speed profile for target velocity {} mm/s".format(target_velocity))
    ax.legend()

    plt.show()


def test_draw_speed():
    t = [i for i in range(11)]
    v = t
    target = 5
    draw_speed(target, t, v)


if __name__ == '__main__':
    test_draw_speed()
