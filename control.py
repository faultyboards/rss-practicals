import copy

IR_MOTION_LIMIT = 350
SONAR_MOTION_LIMIT = 15
MOTION_TIME_LAP = 0


class Control:
    def __init__(self, IO, sensors, motion, robot_position=None):
        self._motion = motion
        self._IO = IO
        self._sensors = sensors
        self._last_state = None
        self._robot_pose = robot_position

    def init_state(self):
        self._last_state = None
        return {
            "poi_detected": False,
            "whisker_on": False,
            "full_on_right": False,
            "full_on_left": False,
            "sonar_on": False,
            "obstacle_right": False,
            "obstacle_left": False
        }

    @staticmethod
    def _reset_other_vars(state, var=None):
        for key in state:
            if key != var:
                state[key] = False

    def sense(self, state):
        new_state = copy.deepcopy(state)
        self._sensors.update_readings()
        curr_condition = None

        if self._sensors.get_light("front") == "poi":
            new_state["poi_detected"] = True
            curr_condition = "poi_detected"
        elif self._sensors.get_whisker():
            # we are facing an obstacle
            new_state["whisker_on"] = True
            curr_condition = "whisker_on"
        elif "whisker_on" in new_state and new_state["whisker_on"]:
            if self._sensors.get_ir("left", True) <= self._sensors.get_ir("right", True):
                new_state["full_on_left"] = True
                curr_condition = "full_on_left"
            else:
                new_state["full_on_right"] = True
                curr_condition = "full_on_right"
        elif self._sensors.get_sonar(True) <= SONAR_MOTION_LIMIT:
            new_state["sonar_on"] = True
            curr_condition = "sonar_on"
        elif self._sensors.get_ir("left", True) >= IR_MOTION_LIMIT:
            new_state["obstacle_left"] = True
            curr_condition = "obstacle_left"
        elif self._sensors.get_ir("right", True) >= IR_MOTION_LIMIT:
            new_state["obstacle_right"] = True
            curr_condition = "obstacle_right"

        # set to False all the conditions different from curr_condition to False
        # If curr_condition is None, resets all the variables
        self._reset_other_vars(new_state, curr_condition)

        return new_state

    def act_navigation(self, state):
        if self._last_state is None or self._last_state != state:
            if state["poi_detected"]:
                self._IO.setStatus('flash', 2, 6, 0)
                self._motion.stop()
            elif state["whisker_on"]:
                # we are facing an obstacle
                self._motion.move(-0.05)
            elif state["sonar_on"]:
                # obstacle behind the robot
                self._motion.stop()
            elif state["obstacle_left"]:
                # something on the left
                self._motion.turn(90, "degrees")
                self._motion.move(0.05)
            elif state["obstacle_right"]:
                # something on the right
                self._motion.turn(-90, "degrees")
                self._motion.move(0.05)
            elif state["full_on_right"]:
                self._motion.turn(15, "degrees")
                self._motion.move(0.05)
            elif state["full_on_left"]:
                self._motion.turn(-15, "degrees")
                self._motion.move(0.05)
            else:
                # go straight
                self._motion.move(0.5)
            self._last_state = state

    def act_poi(self, state, move_dist):
        if self._last_state is None or self._last_state != state:
            if state["poi_detected"]:
                self._IO.setStatus('flash', 2, 6, 0)
                self._motion.stop()
            elif state["whisker_on"]:
                # we are facing an obstacle
                self._motion.move(-0.05)
            elif state["sonar_on"]:
                # obstacle behind the robot
                self._motion.stop()
            elif state["obstacle_left"]:
                # something on the left
                self._motion.turn(90, "degrees")
                self._motion.move(0.05)
            elif state["obstacle_right"]:
                # something on the right
                self._motion.turn(-90, "degrees")
                self._motion.move(0.05)
            elif state["full_on_right"]:
                self._motion.turn(15, "degrees")
                self._motion.move(0.05)
            elif state["full_on_left"]:
                self._motion.turn(-15, "degrees")
                self._motion.move(0.05)
            else:
                # go straight
                self._motion.turn(15)
                self._motion.move(move_dist)
            self._last_state = state
