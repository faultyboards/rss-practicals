import copy

IR_MOTION_LIMIT = 350
SONAR_MOTION_LIMIT = 15
MOTION_TIME_LAP = 0


class Control:
    def __init__(self, sensors, motors):
        self._motors = motors
        self._sensors = sensors
        self._last_state = None

    def sense(self, state):
        new_state = copy.deepcopy(state)
        self._sensors.update_readings()
        print(self._sensors.get_sonar(True))
        if self._sensors.get_whisker():
            # we are facing an obstacle
            new_state["whisker_on"] = True
        elif self._sensors.get_sonar(True) <= SONAR_MOTION_LIMIT:
            new_state["sonar_on"] = True
        elif self._sensors.get_ir("left", True) >= IR_MOTION_LIMIT:
            new_state["obstacle_left"] = True
        elif self._sensors.get_ir("right", True) >= IR_MOTION_LIMIT:
            new_state["obstacle_right"] = True
        else:
            new_state["obstacle_right"] = False
            new_state["obstacle_left"] = False
            new_state["whisker_on"] = False
            new_state["sonar_on"] = False
        return new_state

    def act(self, state):
        print(state)

        if self._last_state != state:
            if state["whisker_on"]:
                # we are facing an obstacle
                self._motors.full_on("backward")
            elif state["sonar_on"]:
                # obstacle behind the robot
                self._motors.stop()
            elif state["obstacle_left"]:
                # something on the left
                self._motors.full_on("right")
            elif state["obstacle_right"]:
                # something on the right
                self._motors.full_on("left")
            else:
                # go straight
                self._motors.full_on("forward")

        self._last_state = state
