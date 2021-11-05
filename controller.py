class Controller:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.accumulated_error = 0

    def control(self, error):
        self.accumulated_error += error
        print("Accumulated error is " + str(self.accumulated_error))
        return self.kp * error + self.ki * self.accumulated_error
