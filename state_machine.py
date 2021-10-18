from biorobotics import Ticker

from states import State
from sensor import SensorState
from state_functions import StateFunctions

class StateMachine():
    def __init__(self, ticker_freq):
        self.robot_state = State()
        self.sensor_state = SensorState()
        self.state_functions = StateFunctions()
        self.ticker = Ticker(0, ticker_freq, self.run)

    def start(self):
        self.ticker.start()

    def run(self):
        self.sensor_state.update()
        self.state_functions.callbacks[self.robot_state.current]()

    def stop(self):
        self.ticker.stop()