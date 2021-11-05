from biorobotics import AnalogIn, Encoder, SerialPC
from pins import Pins
from blueswitch import Blueswitch
from biquad_filter import Biquad

class SensorState:
    def __init__(self, listlowpass, gainlowpass, listbandstop, gainbandstop, listhighpass):
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
        self.highpassfilt_1 = Biquad(listhighpass)
        self.highpassfilt_2 = Biquad(listhighpass)

        self.emg1 = AnalogIn(Pins.EMG_1)
        self.emg2 = AnalogIn(Pins.EMG_2)

        self.potmeter1 = AnalogIn(Pins.POTMETER_1)
        self.potmeter2 = AnalogIn(Pins.POTMETER_2)
        self.blueswitch = Blueswitch()

        self.pc = SerialPC(4)

    def update(self, USE_POTMETERS):
        
        self.switch_value = self.blueswitch.value()
        # print("encoder 1 is " + str(self.pins_encoder_1.counter() - self.encoder_def_1))
        # print("encoder 2 is " + str(self.pins_encoder_2.counter() - self.encoder_def_2))
        self.motor1_sensor = self.pins_encoder_1.counter() - self.encoder_def_1
        self.motor2_sensor = self.pins_encoder_2.counter() - self.encoder_def_2

            # TODO: read sensor values and update them.
            # TODO: add encoder to this.

        if USE_POTMETERS:
            self.emg1_value = self.potmeter1.read()
            self.emg2_value = self.potmeter2.read()
            self.emg1_f = (self.emg1_value - 0.5) * 2
            self.emg2_f = (self.emg2_value - 0.5) * 2
        else:

            self.emg1_value = self.emg1.read()
            self.emg2_value = self.emg2.read()

            # self.pc.set(0, self.emg1_value)
            # self.pc.set(1, self.emg2_value)


            tf_1 = self.highpassfilt_1.filter(self.emg1_value)
            tf_2 = self.highpassfilt_2.filter(self.emg2_value)

            tf_1 = self.gain_1 * self.bandstopfilt_1.filter(tf_1)
            tf_2 = self.gain_1 * self.bandstopfilt_2.filter(tf_2)
            
            tf_1 = abs(tf_1)
            tf_2 = abs(tf_2)

            self.emg1_f = self.lowpassfilt_1.filter(tf_1) * self.gain_2
            self.emg2_f = self.lowpassfilt_2.filter(tf_2) * self.gain_2

            # self.pc.set(2, self.emg1_f)
            # self.pc.set(3, self.emg2_f)

            # self.pc.send()


