import time


class PidCtrl:
    def __init__(self, kp, ki, kd, setpoint=None, sampling_ms=0.01, output_limit=(None, None)):
        """
        Initialize the PID controller
        :param kp: Proportional gain
        :param ki: Integrative gain
        :param kd: Derivative gain
        :param setpoint: Initial setpoint
        :param sampling_ms: Controller sampling
        :param output_limit: Output saturation
        """
        self.active = False

        # Controller parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sampling = sampling_ms
        self.output_limit = output_limit

        # Controller inputs
        self.pv = 0
        self.set_point = setpoint

        # Controller internals
        self.e_prev = 0
        self.e = 0

        # Controller outputs
        self.p = 0
        self.i = 0
        self.d = 0

    @property
    def output(self):
        """
        Define composition of controller output
        :return:
        """
        return self.p + self.i + self.d

    def update_setpoint(self, set_point):
        """
        Update the set point of the controller
        :param set_point:
        :return:
        """
        self.set_point = set_point

    def tune(self, *args):
        """
        Update the controller parameters
        :param args:
        :return:
        """
        self.kp, self.ki, self.kd = args

    def update_input(self):
        self.pv = 0
        raise NotImplementedError

    def update_output(self):
        self.e = self.set_point - self.pv

        self.p = self.kp * self.e
        self.d = self.kd * (self.e - self.e_prev) / self.sampling

        # Anti-Windup
        if self.output_limit[0] is None or self.output_limit[0] <= self.output:
            if self.output_limit[-1] is None or self.output <= self.output_limit[-1]:
                self.i += self.ki * self.e * self.sampling

        self.e_prev = self.e

        # Output saturation
        return min(max(self.output, self.output_limit[-1]), self.output_limit[0])

    def mainloop(self):
        self.active = True

        while self.active:
            self.update_input()
            self.update_output()
            time.sleep(self.sampling / 1000)
