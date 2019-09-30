from GCmd import GCmd
from TestGCmd import TestGCmd


# from MelfaCMD import MelfaCMD


if __name__ == '__main__':
    GCmd.DIGITS = 3
    sample_commands = [
        GCmd("G02", abs_cr=(10.35, -100, None), rel_cr=(0.0, -12, None), speed=30.0, f_speed=20.0),
        GCmd('G04', time_ms=3.0),
        GCmd('G17'),
        GCmd('G18'),
        GCmd('G19'),
        GCmd('G20'),
        GCmd('G21'),
        GCmd('G28', home='Z'),
        GCmd('G90'),
        GCmd('G91'),
    ]

    for command in sample_commands:
        print(command, flush=True)

    # Test string input conversion
    TestGCmd.test_str()
