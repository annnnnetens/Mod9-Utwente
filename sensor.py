from biorobotics import AnalogIn, SerialPC, Encoder
from pins import Pins
from BlueSwitch import BlueSwitch
from biquad_filter import Biquad


class SensorState:
    def __init__(self, listlowpass, gainlowpass, listbandstop, gainbandstop):
        self.emg1_value = 0
        self.emg2_value = 0
        self.motor1_sensor = 0
        self.motor2_sensor = 0
        self.switch_value = 0
        
        # define encoders for motors 1 and 2
        self.pins_encoder_1 = Encoder(Pins.Encoder_1_A, Pins.Encoder_1_B)
        self.pins_encoder_2 = Encoder(Pins.Encoder_2_A, Pins.Encoder_2_B)

        # for filtering the EMG
        self.LP_100_10_1 = Biquad(listlowpass)
        self.gain_1 = gainlowpass
        self.LP_100_10_2 = Biquad(listbandstop)
        self.gain_2 = gainbandstop

        # need this PC to write to uscope
        self.pc = SerialPC(Pins.SERIAL_PC)

        self.emg1 = AnalogIn(Pins.EMG_1)
        self.emg2 = AnalogIn(Pins.EMG_2)

        self.potmeter1 = AnalogIn(Pins.POTMETER_1)
        self.potmeter2 = AnalogIn(Pins.POTMETER_2)
        self.blueswitch = BlueSwitch()

    def update(self, USE_POTMETERS=True):
        
        self.switch_value = self.blueswitch.value()
        self.motor1_sensor = self.pins_encoder_1.counter()
        self.motor2_sensor = self.pins_encoder_2.counter()

            # TODO: read sensor values and update them.
            # TODO: add encoder to this.

        if USE_POTMETERS:
            self.emg1_value = self.potmeter1.read()
            self.emg2_value = self.potmeter2.read()
        else:
            # TODO: the values need to be adjusted to be in range of [0, 1] or something
            # TODO: why is the filter implemented in send_to_pc? where does it come into play
            self.emg1_value = self.emg1.read()
            self.emg2_value = self.emg2.read()


    def send_to_pc(self):
        # copied from low end emg practical
        # to use for graphing in uscope, maybe needs a ticker?
        # doubles over the emg.read() with update() but since this is only for debugging, it should be okay
        
        for i, emg in enumerate(self.emgs): 
            self.pc.set(i, emg.read())
            # TODO: ask about whether emgs = [AnalogIn('A0'), AnalogIn('A1')] takes a single emg two times or two emgs
        
        self.pc.set(2, self.gain_1*self.LP_100_10_1.filter(emg.read()))
        self.pc.send()