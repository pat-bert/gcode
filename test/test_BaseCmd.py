import pytest

from src.BaseCmd import BaseCmd


@pytest.fixture
def base():
    return BaseCmd()


class TestBaseCmd:
    @pytest.mark.parametrize(
        "descriptor,value,delimiter,expected_result",
        [
            ("Test", 3, " ", "Test 3 "),
            ("Test", 3, ":", "Test:3 "),
            ("Test", None, "_", ""),
        ],
    )
    def test_combine(self, descriptor, value, delimiter, expected_result, base):
        assert expected_result == base.combine(descriptor, value, delimiter=delimiter)

    def test_combine_default(self, base):
        assert "KeyValue " == base.combine("Key", "Value")

    def test__is_valid(self, base):
        """
        Abstract class should always return True
        :return:
        """
        assert base._is_valid()

    def test_read_cmd_str(self, base):
        with pytest.raises(NotImplementedError):
            base.read_cmd_str("Test")

    def test_validate(self, base):
        assert base.validate()

    def test_string(self, base):
        with pytest.raises(NotImplementedError):
            str(base)
