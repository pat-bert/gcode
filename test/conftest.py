import logging

import pytest

from src.prechecks.configs import melfa_rv_4a

logging.basicConfig(level=logging.INFO, format='%(levelname)s:%(asctime)s %(message)s',
                    datefmt='%d/%m/%Y %H:%M:%S')


@pytest.fixture
def dh_melfa_rv_4a():
    """
    Provide the DH config for the Mitsubishi Melfa RV-4A
    :return:
    """
    return melfa_rv_4a(rtoff=0, atoff=0)


def pytest_addoption(parser):
    parser.addoption("--runslow", action="store_true", default=False, help="run slow tests")


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
