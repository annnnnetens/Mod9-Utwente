class Controller:
    def __init__(self, sampling_frequency, kp=1, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_previous = 0
        self.accumulated_error = 0
        self.frequency = sampling_frequency

    def transfer(self, error):
        """Calculate transfer function from input+error to output
        input should be the error so r - y (reference - output)
        r is derived from the EMG part
        y should be stored from previous calculations
        The transfer function is in lecture slides of discretization slide 61"""
        self.accumulated_error += error / self.frequency
        self.error_change = (error - self.error_previous) * self.frequency
        controlled_output = self.kp * error + self.ki * self.accumulated_error + self.kd * self.error_change
        self.error_previous = error


        return controlled_output
