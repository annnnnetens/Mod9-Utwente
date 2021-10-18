from states import State


# The statefunctions class can be extended by having the servo motor hold in a certain position.
# This position needs to be visible in all states (use a self. variable)
# and needs to be send to the servo motor at all states (except calibrating).
# Also can be extended by ramping up the speed of the servo motor when lifting/lowering the pencil
class StateFunctions:
    def __init__(self, robot_state, sensor_state):
        self.robot_state = robot_state
        self.sensor_state = sensor_state
        self.callbacks = {
            State.CALIBRATING: self.calibrating,
            State.STAND_BY: self.standby,
            State.LIFTING: self.lifting,
            State.LOWERING: self.lowering,
            State.MOVING: self.moving
        }
        return

    def calibrating(self):
        # TODO: start the calibration (entry action)
        # TODO: keep on calibrating
        # TODO: Find a way to know that calibration has stopped
        # TODO: if done continue to standby
        # EMG also needs to be calibrated right? What should be done over there?
        pass

    def standby(self):
        # TODO: state action should be to send 0 signals to the motors
        # TODO: add state guards that listens for the EMG signals (if it exceeds certain values then go to corresponding state)
        pass

    def lifting(self):
        # TODO: Entry action = stop the rotating motors
        # TODO: state action = send full power to servo motor
        pass

    def lowering(self):
        # TODO: Entry action = stop the rotating motors
        # TODO: state action = send no power to servo motor
        pass

    def moving(self):
        # TODO: state action: calculate using inverse kinematics what the joint rotation should be in order to move the end effector
        # TODO: use the joint rotation results and send that to the motor
        pass
