import pytest

from BaseCmd import BaseCmd


class TestBaseCmd:
    @pytest.mark.parametrize("descriptor,value,delimiter,expected_result",
                             [('Test', 3, ' ', 'Test 3 '), ('Test', 3, ':', 'Test:3 '), ('Test', None, '_', '')])
    def test_combine(self, descriptor, value, delimiter, expected_result):
        a = BaseCmd()
        assert expected_result == a.combine(descriptor, value, delimiter=delimiter)

    def test_combine_default(self):
        a = BaseCmd()
        assert 'KeyValue ' == a.combine('Key', 'Value')

    def test__is_valid(self):
        """
        Abstract class should always return True
        :return:
        """
        a = BaseCmd()
        assert a._is_valid()

    def test_read_cmd_str(self):
        a = BaseCmd()
        with pytest.raises(NotImplementedError):
            a.read_cmd_str('Test')

    def test_validate(self):
        a = BaseCmd()
        result = a.validate()
        assert result

    def test_string(self):
        a = BaseCmd()
        with pytest.raises(NotImplementedError):
            str(a)
