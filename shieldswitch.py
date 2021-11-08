from pyb import Pin
import micropython
import biorobotics

class Shieldswitch:
    def __init__(self, button):
        self.switch = Pin(button, Pin.IN, Pin.PULL_UP)
        self.previous_value = 1

    def value(self):
        current_val = self.switch.value()
        if current_val - self.previous_value == -1:
            self.previous_value = current_val
            return 1
        else:
            self.previous_value = current_val
            return 0