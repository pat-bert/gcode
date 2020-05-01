from math import pi

import pytest

from src.kinematics.joint_factories import BaseJointFactory


@pytest.fixture
def dh_melfa_rv_4a():
    """
    Provide the DH config for the Mitsubishi Melfa RV-4A
    :return:
    """
    rtoff = 0.0  # radial tool offset
    atoff = 0.0  # axial tool offset

    # Denavit-Hartenberg parameters: a - alpha - d - zero offset
    # Mitsubishi defines axis origins differently than resulting from DH-convention
    dh_parameters = [
        [100, -pi / 2, 350, 0],
        [250, 0.00000, 0.0, -pi / 2],
        [135, -pi / 2, 0.0, -pi / 2],
        [0.0, +pi / 2, 250, 0],
        [0.0, -pi / 2, 0.0, 0],
        [rtoff, 0.00000, 90 + atoff, pi]
    ]

    config = [BaseJointFactory.new(a=a, alpha=alpha, d=d, theta=None, offset=z) for a, alpha, d, z in dh_parameters]
    return config


def pytest_addoption(parser):
    parser.addoption(
        "--runslow", action="store_true", default=False, help="run slow tests"
    )


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: mark test as slow to run")


def pytest_collection_modifyitems(config, items):
    if config.getoption("--runslow"):
        # --runslow given in cli: do not skip slow tests
        return
    skip_slow = pytest.mark.skip(reason="need --runslow option to run")
    for item in items:
        if "slow" in item.keywords:
            item.add_marker(skip_slow)
