import numpy as np
from states import State
from motor import Motor
from pins import Pins
import machine
from RKI import jacobian_angles, unit_twist_rotational, brockett


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
        self.q1 = 0
        self.q2 = 0
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
            self.robot_state.set(State.STAND_BY)

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
        EMG_signal_1 = self.sensor_state.EMG_biceps
        EMG_signal_2 = self.sensor_state.EMG_triceps
        # Transformed signal
        transformed_signal_1 = (EMG_signal_1 - 0.5) * 2
        transformed_signal_2 = (EMG_signal_2 - 0.5) * 2
        print("writing " + str(transformed_signal_1) + " to motors")
        self.motor_joint_base.write(transformed_signal_1)
        self.motor_joint_arm.write(transformed_signal_1)
        # TODO: need to add position or velocity to the function below instead of 0, 0.4
        dq = self.calculate_dq(0, 0.4)
        # TODO: need to incorparate the dq somewhere in the motors
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
        EMG_signal_1 = self.sensor_state.EMG_biceps
        EMG_signal_2 = self.sensor_state.EMG_triceps

        print("signal is " + str(EMG_signal_1) + " and blueswitch is " + str(switch_val))
        if switch_val == 1 and self.robot_state.current != State.TOGGLING:
            self.robot_state.set(State.TOGGLING)
            print("going to toggling")
        # Just using stub values here. Feel free to change
        elif EMG_signal_1 > 0.75 or EMG_signal_1 < 0.25 or EMG_signal_2 > 0.75 or EMG_signal_2:
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

    def calculate_dq(self, x_desired, y_desired):
        reference_configuration = np.array([
            [1, 0, 0],
            [0, 1, 0.425],
            [0, 0, 1]
        ])
        reference_twist1 = unit_twist_rotational(0, 0)
        reference_twist2 = unit_twist_rotational(0, 0.24)
        He0 = brockett(reference_configuration, (reference_twist1, self.q1), (reference_twist2, self.q2), degrees=True)
        J = jacobian_angles(self.q1)

        pe0 = He0[:2, 2]
        ps = np.array([x_desired, y_desired])  # TODO: insert desired position or velocity?
        K = 10  # TODO: tune K to have good response
        F = K * (ps - pe0)
        # TODO: maybe constrict F like is done in exercise E
        Ws0 = np.array([pe0[0] * F[1] - pe0[1] * F[0], F[0], F[1]])
        tau = J.T @ Ws0

        b = [0.5, 0.25]  # TODO: tune b to have good response
        dq = tau / b
        return dq
