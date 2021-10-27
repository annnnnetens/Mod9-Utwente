import pyb
import micropython
import biorobotics


class BlueSwitch:
    def __init__(self):
        self.switch_value = 0
        self.switch = pyb.Switch()
        self.switch.callback(self.callback)
        biorobotics.tic()

    def callback(self):
        micropython.heap_unlock()
        elapsed = biorobotics.toc()
        micropython.heap_lock()
        if elapsed > 250000:
            self.switch_value = 1
            biorobotics.tic()

    def value(self):
        res = self.switch_value
        self.switch_value = 0
        return res
