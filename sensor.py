from biorobotics import AnalogIn
import pins

class SensorState:
    def __init__(self):
        # TODO: what sensors are available to us?
        # Not sure if we are reading biceps and triceps. Feel free to change
        self.EMG_biceps = 0
        self.EMG_triceps = 0
        self.motor_sensor = 0
        # We are using potmeters right?
        self.potmeter1 = AnalogIn(pins.Pins.POTMETER_1)
        self.potmeter2 = AnalogIn(pins.Pins.POTMETER_2)

    def update(self):
        # TODO: read sensor values and update them.
        # TODO: add encoder to this.
        self.EMG_biceps = self.potmeter1.read()
        self.EMG_triceps = self.potmeter2.read()
