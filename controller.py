class Controller:
    def __init__(self, kp):
        self.kp = kp

    def control(self, error):
        return self.kp * error
