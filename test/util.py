import pytest


class SkipIfConditionWrapper:
    """
    This class can be instantiated to create a decorator to skip tests for a common condition.
    """

    def __init__(self, validation_func, reason):
        """
        Parametrize the condition.
        :param validation_func:
        :param reason:
        """
        self.is_available = validation_func()
        self.reason = reason

    def required(self, func):
        """
        Decorator method:
        The function will be decorated with the skipif decorator of pytest with the condition and reason filled in.
        :param func: Function to be decorated.
        :return: Decorated function.
        """
        return pytest.mark.skipif(not self.is_available, reason=self.reason)(func)
