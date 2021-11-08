class State:
    CALIBRATING = 'calibrating'
    STAND_BY = 'standby'
    TOGGLING = 'toggling'
    MOVING = 'moving'
    DEMO = 'demo'

    def __init__(self):
        self.previous = None
        self.current = self.CALIBRATING

    def set(self, state):
        self.previous = self.current
        self.current = state

    def is_changed(self):
        return self.current is not self.previous
