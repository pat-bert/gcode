import unittest
from GCmd import GCmd


class TestGCmd(unittest.TestCase):
    CONSOLE_WIDTH = 30
    DELIMITER = '#'*CONSOLE_WIDTH

    @classmethod
    def test_str(cls):
        """
        Tests the creation of objects from a command string and their conversion back to the string.
        :return:
        """
        sample_cmd_strings = {
            'G28 X Y': 'G28 X Y',
            'G28 Y10.0 Z': 'G28 Y Z',
            'G01 X10.3 Z-10.3': 'G1 X10.3 Z-10.3',
            'G01 X-50.0 Y30.0 F30.2 E17.1 S20.3': 'G1 X-50.0 Y30.0 F30.2 E17.1 P20300.0',
            'G02 I5.0 K5.3': 'G2 I5.0 K5.3',
            'G01': 'G1',
            'G28': 'G28',
            'G28 X Y Z': 'G28 X Y Z',
            'M104 S3.0': 'M104 S3.0',
            '; I am a comment.': 'None',
        }

        # Define digits to have outcomes as expected.
        GCmd.DIGITS = 1
        GCmd.CMD_REMOVE_LEAD_ZERO = True

        print(cls.DELIMITER)
        print("Testing string conversion for GCmd")
        print(cls.DELIMITER)

        failed_tests = 0
        for cmd_str, out_trg in sample_cmd_strings.items():
            out_act = ''
            try:
                out_act = str(GCmd.read_cmd_str(cmd_str))
                assert out_act == out_trg
                print("Passed.")
            except AssertionError:
                print("Got: '", out_act, "'")
                print("Expected: '", out_trg, "'")
                failed_tests += 1
            except Exception as e:
                print(e)
                print("Expected: '", out_trg, "'")
                failed_tests += 1

        print("Total failed tests: ", str(failed_tests), "/", str(len(sample_cmd_strings.keys())))
