from states import State
from motor import Motor
from pins import Pins
from biorobotics import PWM, SerialPC
from rki import calculate_dq_j_inv
from controller import Controller
from math import sqrt
try:
    import numpy as np
except ImportError:
    import ulab as np


# The statefunctions class can be extended by having the servo motor hold in a certain position.
# This position needs to be visible in all states (use a self. variable)
# and needs to be send to the servo motor at all states (except calibrating).
# Also can be extended by ramping up the speed of the servo motor when lifting/lowering the pencil
class StateFunctions:

    def __init__(self, robot_state, sensor_state, use_pm, ticker_frequency):
        self.robot_state = robot_state
        self.sensor_state = sensor_state
        self.callbacks = {
            State.CALIBRATING: self.calibrating,
            State.STAND_BY: self.standby,
            State.TOGGLING: self.toggling,
            State.MOVING: self.moving
        }
        self.motor_joint_base = Motor(Pins.MOTOR_1_DIRECTION, Pins.MOTOR_1_PWN, ticker_frequency)
        self.motor_joint_arm = Motor(Pins.MOTOR_2_DIRECTION, Pins.MOTOR_2_PWM, ticker_frequency)
        self.servo_motor = PWM(Pins.SERVO_MOTOR, ticker_frequency)

        # need this PC to write to uscope
        self.pc = SerialPC(Pins.SERIAL_PC)

        self.USE_PM = use_pm

        self.max_emg_1 = 0.01
        self.max_emg_2 = 0.01
        self.max_speed = 0.02

        self.frequency = ticker_frequency
        self.servo_motor_value = 1
        self.q1 = 0
        self.q2 = 0
        self.desired_position = [0.028, 0.425]
        self.controller_dq1 = Controller(kp=3, ki=0.01)
        self.controller_dq2 = Controller(kp=1, ki=0.01)
        return

    def calibrating(self):
        # TODO: EMG also needs to be calibrated. Needs to be hardcoded (although if you do it in this state it would be fun :))
        switch_val = self.sensor_state.switch_value
        if self.robot_state.is_changed():
            print("Starting calibration")
            print("Please move the arm untill it is completely stretched")
            print("If you are done press the blue button")
            self.robot_state.set(self.robot_state.current)
        elif switch_val:
            print("\nCalibration done! Have fun with drawing\n")
            # Reads the encoder values at the instant of pressing the button, so the encoders are "reset to zero"
            self.sensor_state.encoder_def_1 = self.sensor_state.motor1_sensor
            self.sensor_state.encoder_def_2 = self.sensor_state.motor2_sensor
            self.robot_state.set(State.STAND_BY)

    def standby(self):
        print("standing by")

        self.stop_motor()
        self.write_servo_motor()
        self.listen_for_signal()
        pass

    def toggling(self):
        # TODO: synchronize this with frequency of controller
        if self.robot_state.is_changed():
            if self.servo_motor_value == 1:
                self.servo_motor_value = -1
            else:
                self.servo_motor_value = 1
            self.robot_state.set(self.robot_state.current)

        self.stop_motor()
        self.write_servo_motor()
        self.listen_for_signal()

    def moving(self):
        # TODO: state action: calculate using inverse kinematics what the joint rotation should be in order to move the end effector
        # TODO: use the joint rotation results and send that to the motor

        self.q1 = self.sensor_state.motor1_sensor / 131.25 / 64 * 2  # now there are 2 q1 in one rotation
        self.q2 = - self.sensor_state.motor2_sensor / 131.25 / 64 * 2  # q1 and q2 are therefore in radians*pi
        if not self.USE_PM:

            EMG_signal_1 = self.sensor_state.emg1_f
            EMG_signal_2 = self.sensor_state.emg2_f
            transformed_signal_1 = 2 * (EMG_signal_1 - 0.5)
            transformed_signal_2 = 2 * (EMG_signal_2 - 0.5)

            # printing the emgs as graphs in uscope
            self.pc.set(0, self.sensor_state.emg1_value)
            self.pc.set(1, EMG_signal_1)
            self.pc.set(2, self.sensor_state.emg2_value)
            self.pc.set(3, EMG_signal_2)
            self.pc.send()

        else:
            EMG_signal_1 = self.sensor_state.emg1_value
            EMG_signal_2 = self.sensor_state.emg2_value
            transformed_signal_1 = 2 * (EMG_signal_1 - 0.5)
            transformed_signal_2 = 2 * (EMG_signal_2 - 0.5)
        if abs(transformed_signal_1) < 0.2: # 0.2 for anete for 5Hz cutoff, was 0.015
            transformed_signal_1 = 0
        if abs(transformed_signal_2) < 0.2:
            transformed_signal_2 = 0
        # checks for EMG values larger than one and resets them to one. Could be also a bit lower than 1, to saturate
        if abs(transformed_signal_1) > 1:
            transformed_signal_1 = transformed_signal_1 / abs(transformed_signal_1)
        if abs(transformed_signal_2) > 1:
            transformed_signal_2 = transformed_signal_2 / abs(transformed_signal_2)

        # the diagonal values are larger! Need to normalize and only allow :
        # value_speed = sqrt((transformed_signal_1 ** 2 + transformed_signal_2 ** 2)/2)
        # transformed_signal_1 = transformed_signal_1 * self.max_speed / value_speed
        # transformed_signal_2 = transformed_signal_2 * self.max_speed / value_speed
        speed_constant = 0.02
        self.desired_position = [self.desired_position[0] + speed_constant * transformed_signal_1 / self.frequency,
                                 self.desired_position[1] + speed_constant * transformed_signal_2 / self.frequency]

        dq = calculate_dq_j_inv(self.q1, self.q2, self.desired_position[0], self.desired_position[1])
        # TODO: check for physical bounds
        if (self.q1 >= 0.5 and transformed_signal_1 > 0.5) or (self.q1 <= -0.5 and transformed_signal_1 < -0.5):
            transformed_signal_1 = 0.5 * transformed_signal_1 / abs(transformed_signal_1)
            print("Joint 1 has reached it's bounds. Stopping the motor")
        if (self.q2 >= 12.5 / 18 and transformed_signal_2 > 12.5/18) or (self.q2 <= -12.5 / 18 and transformed_signal_2 < -12.5/18):
            transformed_signal_2 = 12.5/18 * transformed_signal_2 / abs(transformed_signal_2)
            print("Joint 2 has reached it's bounds. Stopping the motor")
        voltage1 = -self.controller_dq1.control(dq[0][0])
        voltage2 = -self.controller_dq2.control(dq[1][0])
        self.motor_joint_base.write(voltage1)
        self.motor_joint_arm.write(voltage2)

        self.write_servo_motor()
        self.listen_for_signal()

    def listen_for_signal(self):
        """
        It seems that in the states standby, lifting, lowering and moving you can move to any of the other states.
        Moreover it needs the same state guards for that.
        So why implement 4 times the same code when it also fits nicely in one function.
        Functionality: reads EMG signal (from the SensorState class).
        If it signals that the pen should be lifted/lowered go to the corresponding state
        If (use elif) the signal says the arm needs to move then change the state to moving
        If no signal then set the state to standby
        """

        switch_val = self.sensor_state.switch_value
        EMG_signal_1 = self.sensor_state.emg1_f
        EMG_signal_2 = self.sensor_state.emg2_f

        # print("signal 1 is " + str(EMG_signal_1) + "signal 2 is " + str(EMG_signal_2) + " and blueswitch is " + str(switch_val))
        if switch_val == 1 and self.robot_state.current != State.TOGGLING:
            self.robot_state.set(State.TOGGLING)
            print("going to toggling")
        # Just using stub values here. Feel free to change
        elif abs(EMG_signal_1) > 0.1 or abs(EMG_signal_2) > 0.1:
            self.robot_state.set(State.MOVING)
            if self.robot_state.is_changed():
                print("going to moving")
        else:
            self.robot_state.set(State.STAND_BY)
            if self.robot_state.is_changed():
                print("going to standby")

    def write_servo_motor(self):
        self.servo_motor.write(0.075 + 0.025 * self.servo_motor_value)

    def stop_motor(self):
        self.motor_joint_arm.write(0)
        self.motor_joint_base.write(0)
