from biorobotics import AnalogIn, Encoder
from pins import Pins
from blueswitch import Blueswitch
from biquad_filter import Biquad

class SensorState:
    def __init__(self, listlowpass, gainlowpass, listbandstop, gainbandstop):
        self.emg1_value = 0
        self.emg1_f = 0
        self.emg2_f = 0
        self.emg2_value = 0
        self.motor1_sensor = 0
        self.motor2_sensor = 0
        self.switch_value = 0
        self.encoder_def_1 = 0
        self.encoder_def_2 = 0
        
        # define encoders for motors 1 and 2
        self.pins_encoder_1 = Encoder(Pins.Encoder_1_A, Pins.Encoder_1_B)
        self.pins_encoder_2 = Encoder(Pins.Encoder_2_A, Pins.Encoder_2_B)

        # for filtering the EMG
        self.lowpassfilt_1 = Biquad(listlowpass)
        self.lowpassfilt_2 = Biquad(listlowpass)
        self.gain_1 = gainlowpass
        self.bandstopfilt_1 = Biquad(listbandstop)
        self.bandstopfilt_2 = Biquad(listbandstop)
        self.gain_2 = gainbandstop

        self.emg1 = AnalogIn(Pins.EMG_1)
        self.emg2 = AnalogIn(Pins.EMG_2)

        self.potmeter1 = AnalogIn(Pins.POTMETER_1)
        self.potmeter2 = AnalogIn(Pins.POTMETER_2)
        self.blueswitch = Blueswitch()

    def update(self, USE_POTMETERS=True):
        
        self.switch_value = self.blueswitch.value()
        self.motor1_sensor = self.pins_encoder_1.counter() - self.encoder_def_1
        self.motor2_sensor = self.pins_encoder_2.counter() - self.encoder_def_2

            # TODO: read sensor values and update them.
            # TODO: add encoder to this.

        if USE_POTMETERS:
            self.emg1_value = self.potmeter1.read()
            self.emg2_value = self.potmeter2.read()
        else:
            # TODO: the values need to be adjusted to be in range of [0, 1] or something

            self.emg1_value = self.emg1.read()
            self.emg2_value = self.emg2.read()

            self.emg1_value = self.emg1_value - 0.46
            self.emg2_value = self.emg2_value - 0.46
            
            tf_1 = self.bandstopfilt_1.filter(self.emg1_value) * self.gain_1 
            tf_2 = self.bandstopfilt_2.filter(self.emg2_value) * self.gain_1
            
            tf_1 = abs(tf_1)
            tf_2 = abs(tf_2)

            tf_1 = self.lowpassfilt_1.filter(tf_1) * self.gain_2
            tf_2 = self.lowpassfilt_2.filter(tf_2) * self.gain_2
            


            # tf_1 = 2 * (tf_1 - 0.5)
            # tf_2 = 2 * (tf_2 - 0.5)

            self.emg1_f = tf_1
            self.emg2_f = tf_2
            print('printing values')
            print(self.emg1_f, self.emg1_value) 
            print(self.emg2_f, self.emg2_value) 




