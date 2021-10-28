class Controller:
    def __init__(self, sampling_frequency):
        self.kp = 1
        self.kd = 1
        self.ki = 1
        self.sampling_frequency = sampling_frequency
        self.previous_input = 0
        self.previous_output = 0

    def transfer(self, transfer_input):
        """Calculate transfer function from input+error to output
        input should be the error so r - y (reference - output)
        r is derived from the EMG part
        y should be stored from previous calculations
        The transfer function is in lecture slides of discretization slide 61"""
        kd = self.kd
        ki = self.ki
        ts = self.sampling_frequency
        controlled_output = self.kp * (((2 * kd + ts) / (2 * ki + ts)) * transfer_input
                                       - ((2 * kd - ts) / (2 * ki - ts) * self.previous_input)) \
            + (2 * ki - ts) / (2 * ki + ts) * self.previous_output
        self.previous_input = transfer_input
        self.previous_output = controlled_output
        return controlled_output
