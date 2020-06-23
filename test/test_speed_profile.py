from math import ceil

import pytest

from src.prechecks.speed_profile import trapezoidal_speed_profile


@pytest.mark.parametrize("acc,vmax,ds,s_total,t_expected",
                         [
                             # Regular case (11 points)
                             (1, 2, 1, 10, [0, 1.41, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.59, 7]),
                             # ds = s_total
                             (1, 2, 10, 10, [0, 7]),
                             # ds > s_total
                             (1, 2, 11, 10, [0, 7])
                         ]
                         )
def test_trapezoidal_speed_profile(acc, vmax, ds, s_total, t_expected):
    t_actual = trapezoidal_speed_profile(vmax, acc, s_total, ds)
    assert pytest.approx(t_actual, abs=0.1) == t_expected
    assert len(t_actual) == ceil(s_total / ds) + 1
