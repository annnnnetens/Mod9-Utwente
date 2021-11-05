from biorobotics import Ticker

from states import State
from sensor import SensorState
from state_functions import StateFunctions


class StateMachine:
    def __init__(self, ticker_freq):
        # 6Hz cut off, sample frequency 1000 Hz, order 2 Butterworth low pass  - for EMG
        listlowpass = [-1.9467,   0.9481, 0.000346, 0.0006921, 0.000346] # quite smooth!
        gainlowpass = 1
        # listlowpass = [0.3695, 0.1958, 0.3913, 0.7827, 0.3913] # really noisy
        # listlowpass = [-0.7478, 0.2722, 0.1311, 0.2622, 0.1311]
        # 48-52 band stop Butterworth - for EMG
        listbandstop = [-0.960616192564186621716260106040863320231,  0.919547137907040124105151335243135690689, 1.0, -1.000877940003687793790732030174694955349, 1.0]
        gainbandstop = 1/0.06

        listhighpass = [-1.889, 0.8949, 0.9460, -1.8920, 0.9460]




        self.USE_POTMETERS = True

        self.robot_state = State()
        self.sensor_state = SensorState(listlowpass, gainlowpass, listbandstop, gainbandstop, listhighpass)
        self.state_functions = StateFunctions(self.robot_state, self.sensor_state, self.USE_POTMETERS ,ticker_freq)
        self.ticker = Ticker(0, ticker_freq, self.run)

    def start(self):
        self.ticker.start()

    def run(self):
        self.sensor_state.update(self.USE_POTMETERS) # TODO: add False to measure EMGs and not potmeters
        self.state_functions.callbacks[self.robot_state.current]()
        # can use self.sensor_state.send_to_pc()

    def stop(self):
        self.ticker.stop()
