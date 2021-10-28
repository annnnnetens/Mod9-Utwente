from biorobotics import Ticker

from states import State
from sensor import SensorState
from state_functions import StateFunctions
from emg_reading import EMG_read


class StateMachine:
    def __init__(self, ticker_freq):
        # 10Hz cut off Butterworth low pass - for EMG
        listlowpass = [-1.705552145544083852968242354108951985836,   0.743655195048865791385139800695469602942, 1.0, 2.0, 1.0]
        gainlowpass = 0.009525762376195455113925270040908799274
        # 48-52 band stop Butterworth - for EMG
        listbandstop = [-0.960616192564186621716260106040863320231,  0.919547137907040124105151335243135690689, 1.0, -1.000877940003687793790732030174694955349, 1.0]
        gainbandstop = 0.959773568953520062052575667621567845345

        self.robot_state = State()
        self.sensor_state = SensorState(listlowpass, gainlowpass, listbandstop, gainbandstop)
        self.state_functions = StateFunctions(self.robot_state, self.sensor_state, ticker_freq)
        self.ticker = Ticker(0, ticker_freq, self.run)
        # self.emgs = EMG_read(list_bw_lowpass, gain_bw_lowpass, list_bw_bandstop, gain_bw_bandstop)

    def start(self):
        self.ticker.start()

    def run(self):
        self.sensor_state.update()
        self.state_functions.callbacks[self.robot_state.current]()

    def stop(self):
        self.ticker.stop()
