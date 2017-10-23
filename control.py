import copy

IR_MOTION_LIMIT = 0.10
SONAR_MOTION_LIMIT = 0.10
MOTION_TIME_LAP = 5


class Control:
    def __init__(self, sensors, motors):
        self._motors = motors
        self._sensors = sensors

    def sense(self, state):
        new_state = copy.deepcopy(state)
        self._sensors.update_readings()
        if self._sensors.get_whisker():
            # we are facing an obstacle
            new_state["whisker_on"] = True
        elif self._sensors.get_sonar() <= SONAR_MOTION_LIMIT:
            new_state["sonar_on"] = True
        elif self._sensors.get_ir("left") <= IR_MOTION_LIMIT:
            new_state["obstacle_left"] = True
        elif self._sensors.get_ir("right") <= IR_MOTION_LIMIT:
            new_state["obstacle_right"] = True
        else:
            new_state["obstacle_right"] = False
            new_state["obstacle_left"] = False
            new_state["whisker_on"] = False
            new_state["sonar_on"] = False

        return new_state

    def act(self, state):
        if state["whisker_on"]:
            # we are facing an obstacle
            self._motors.full_on("backward", MOTION_TIME_LAP)
        elif state["sonar_on"]:
            # obstacle behind the robot
            self._motors.stop()
        elif state["obstacle_left"]:
            # something on the left
            self._motors.full_on("right", MOTION_TIME_LAP)
        elif state["obstacle_right"]:
            # something on the right
            self._motors.full_on("left", MOTION_TIME_LAP)
        else:
            # go straight
            self._motors.full_on("forward", MOTION_TIME_LAP)
