# TODO: add pin defintions
# Encoder (two for each motor)
# Two motors and one servo
# EMG pins
# Potmeter
class Pins:
    MOTOR_1_DIRECTION = 'D4'
    MOTOR_1_PWN = 'D5'
    MOTOR_2_DIRECTION = 'D7'
    MOTOR_2_PWM = 'D6'

    POTMETER_1 = 'A5'
    POTMETER_2 = 'A4'
    SERVO_MOTOR = 'D2'

    ENCODER_1_A = 'D1'
    ENCODER_1_B = 'D0'
    ENCODER_2_A = 'D12'
    ENCODER_2_B = 'D11'

    SERIAL_PC = 3 # only for graphing the EMG, not needed in the end
    EMG_1 = 'A0'
    EMG_2 = 'A1'

