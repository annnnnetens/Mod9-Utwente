from biorobotics import AnalogIn, SerialPC
from pins import Pins
from biquad_filter import Biquad

class EMG_read(object):

    def __init__(self, b_lowpass, lowpass_gain, b_bandstop, bandstop_gain):
        # Initialize filters obtained from MATLAB's filterdesigner 

        self.LP_100_10_1 = Biquad(b_lowpass)
        self.gain_1 = lowpass_gain
        self.LP_100_10_2 = Biquad(b_bandstop)
        self.gain_2 = bandstop_gain

        self.pc = SerialPC(Pins.SERIAL_PC)

        self.emg1 = AnalogIn(Pins.EMG_1)
        self.emg2 = AnalogIn(Pins.EMG_2)

        self.emg1_value = 0
        self.emg2_value = 0


    def send_to_pc(self):
        
        for i, emg in enumerate(self.emgs):
            self.pc.set(i, emg.read())
            
        
        self.pc.set(2, self.gain_1*self.LP_100_10_1.filter(emg.read()))
        self.pc.send()


    def read(self):
        # updates the EMG values
        self.emg1_value = self.emg1.read()
        self.emg2_value = self.emg2.read()


