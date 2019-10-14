from typing import Union


class MxtCmd:
    # For command and monitoring
    MXT_TYPE_POSE = 1
    MXT_TYPE_JOINT = 2
    MXT_TYPE_PULSE = 3

    # Variables related to positions
    MXT_TYPE_FPOSE = 4
    MXT_TYPE_FJOINT = 5
    MXT_TYPE_FPULSE = 6
    MXT_TYPE_FB_POSE = 7
    MXT_TYPE_FB_JOINT = 8
    MXT_TYPE_FB_PULSE = 9

    # Variables related to current
    MXT_TYPE_CMDCUR = 10
    MXT_TYPE_FBKCUR = 11


class Pose:
    pass


class Joint:
    pass


class Pulse:
    pass


class WholeNum:
    pass


class RTData:
    def __init__(self, rt_data: Union[Pose, Joint, Pulse, WholeNum]):
        self.data = rt_data
