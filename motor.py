from biorobotics import PWM
from machine import Pin


class Motor:
    def __init__(self, motordir, motorpwm, frequency):
        self.pwm_dir = Pin(motordir, Pin.OUT)
        self.pwm_vel = PWM(motorpwm, frequency)
        return

    def write(self, value):
        if value < 0:
            self.pwm_dir.value(1)
        else:
            self.pwm_dir.value(0)

        self.pwm_vel.write(abs(value))
