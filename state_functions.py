from states import State
from motor import Motor
from pins import Pins
import machine

# The statefunctions class can be extended by having the servo motor hold in a certain position.
# This position needs to be visible in all states (use a self. variable)
# and needs to be send to the servo motor at all states (except calibrating).
# Also can be extended by ramping up the speed of the servo motor when lifting/lowering the pencil
class StateFunctions:
    def __init__(self, robot_state, sensor_state, ticker_frequency):
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
        self.servo_motor = machine.Pin(Pins.SERVO_MOTOR, machine.Pin.OUT)
        self.servo_motor_value = 0
        return

    def calibrating(self):
        # TODO: start the calibration (entry action)
        # TODO: keep on calibrating
        # TODO: Find a way to know that calibration has stopped
        # If a signal for lowering the pencil is given than then calibration is done
        # TODO: if done continue to lowering
        # EMG also needs to be calibrated right? What should be done over there?
        pass

    def standby(self):
        print("standing by")

        self.stop_motor()
        self.write_servo_motor()
        self.listen_for_signal()
        pass

    def toggling(self):
        self.servo_motor_value = not self.servo_motor_value
        print("servo motor is now " + str(self.servo_motor_value))

        self.stop_motor()
        self.write_servo_motor()
        self.listen_for_signal()

    def moving(self):
        # TODO: state action: calculate using inverse kinematics what the joint rotation should be in order to move the end effector
        # TODO: use the joint rotation results and send that to the motor
        EMG_signal = self.sensor_state.EMG_triceps
        transformed_signal = (EMG_signal - 0.5) * 2
        print("writing " + str(transformed_signal) + " to motors")
        self.motor_joint_base.write(transformed_signal)
        self.motor_joint_arm.write(transformed_signal)

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
        EMG_signal_1 = self.sensor_state.EMG_triceps
        print("signal is " + str(EMG_signal_1) + " and blueswitch is " + str(switch_val))
        # Just using stub values here. Feel free to change
        if switch_val == 1 and self.robot_state.current != State.TOGGLING:
            self.robot_state.set(State.TOGGLING)
        elif EMG_signal_1 > 0.75 or EMG_signal_1 < 0.25:
            self.robot_state.set(State.MOVING)
            if self.robot_state.is_changed():
                print("going to moving")
        else:
            self.robot_state.set(State.STAND_BY)
            if self.robot_state.is_changed():
                print("going to standby")

    def write_servo_motor(self):
        self.servo_motor.value(self.servo_motor_value)

    def stop_motor(self):
        self.motor_joint_arm.write(0)
        self.motor_joint_base.write(0)
