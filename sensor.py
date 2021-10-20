class SensorState:
    def __init__(self):
        # TODO: what sensors are available to us?
        # Not sure if we are reading biceps and triceps. Feel free to change
        self.EMG_biceps = 0
        self.EMG_triceps = 0
        self.motor_sensor = 0
        # We are using potmeters right?
        self.potmeter = 0

        pass

    def update(self):
        # TODO: read sensor values and update them.
        # TODO: add encoder to this.
        pass
