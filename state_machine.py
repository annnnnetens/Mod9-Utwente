from biorobotics import Ticker

from states import State
from sensor import SensorState
from state_functions import StateFunctions


class StateMachine:
    def __init__(self, ticker_freq):
        # 6Hz cut off, sample frequency 1000 Hz, order 2 Butterworth low pass  - for EMG
        listlowpass = [-1.9467,   0.9481, 0.000346, 0.0006921, 0.000346]
        gainlowpass = 0.009525762376195455113925270040908799274
        # 48-52 band stop Butterworth - for EMG
        listbandstop = [-0.960616192564186621716260106040863320231,  0.919547137907040124105151335243135690689, 1.0, -1.000877940003687793790732030174694955349, 1.0]
        gainbandstop = 0.959773568953520062052575667621567845345

        self.USE_POTMETERS = False

        self.robot_state = State()
        self.sensor_state = SensorState()
        self.state_functions = StateFunctions(self.robot_state, self.sensor_state, listlowpass, gainlowpass, listbandstop, gainbandstop, self.USE_POTMETERS ,ticker_freq)
        self.ticker = Ticker(0, ticker_freq, self.run)

    def start(self):
        self.ticker.start()

    def run(self):
        self.sensor_state.update(self.USE_POTMETERS) # TODO: add False to measure EMGs and not potmeters
        self.state_functions.callbacks[self.robot_state.current]()
        # can use self.sensor_state.send_to_pc()

    def stop(self):
        self.ticker.stop()
