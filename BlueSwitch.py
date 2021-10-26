import pyb

# TODO: implement catching the button bounce (using tic and toc)
class BlueSwitch:
    def __init__(self):
        self.switch_value = 0
        self.switch = pyb.Switch()
        self.switch.callback(self.callback)

    def callback(self):
        self.switch_value = 1
        return

    def value(self):
        res = self.switch_value
        self.switch_value = 0
        return res