from collections import defaultdict
from heapq import heappush


class Grid:
    # free positions will be stored in the grid
    # if a position is not present in the grid, it won't be accessible by the robot
    def __init__(self, real_width, real_height, robot_width, robot_height, free_positions):
        self._scaled_width = 0
        self._scaled_height = 0
        self._grid = {}

        self._fill_positions(free_positions)

    def _fill_positions(self, free_positions):
        for pos, neighbours in free_positions.items():
            # for each neighbour the cost to get to it is 1
            self._grid[pos] = [(n, 1) for n in neighbours]


class State:
    def __init__(self, sensors, inputs, camera):
        self._camera = camera
        self._sensors = sensors
        self._inputs = inputs

    @property
    def inputs(self):
        return self._inputs

    @property
    def camera(self):
        return self._camera

    @property
    def sensors(self):
        return self._sensors

    def __eq__(self, other):
        return self._inputs == other.inputs and self._camera == other.camera and self._sensors == other.sensors


class Action:
    def __init__(self, action, priority=1):
        self._action = action
        self._priority = priority

    @property
    def action(self):
        return self._action

    @property
    def priority(self):
        return self._priority

    def __cmp__(self, other):
        main_cmp = cmp(self._priority, other.priority)

        return self._action == other.action if main_cmp == 0 else main_cmp

    def __str__(self):
        return 'Action=({}, {})'.format(self._action, self._priority)

    def __repr__(self):
        return self.__str__()


class StateMachine:
    def __init__(self):
        self._state_mapping = defaultdict(list)

    def add_mapping(self, state, action, priority=5):
        heappush(self._state_mapping[state], Action(action, priority))

    def get_action(self, state):
        # retrieves the action with the highest priority
        return self._state_mapping[state]

    def __str__(self):
        str_value = ''
        for state in self._state_mapping:
            str_value += 'State=({}) ->'.format(state)
            str_value += str(self._state_mapping[state])
            str_value += '\n'
        return str_value


class Filter:
    def __call__(self, *inputs):
        raise NotImplementedError


class KalmanFilter(Filter):
    def __call__(self, *inputs):
        raise NotImplementedError


class Control:
    def __init__(self, grid, IO, vision):
        self._grid = grid
        self._IO = IO
        self._filter = KalmanFilter()
        self._vision = vision

    def sense(self):
        inputs = self._IO.getInputs()
        sensors = self._IO.getSensors()
        camera_image = self._vision.get_camera()
        return self._filter(inputs, sensors, camera_image)

    def plan(self):
        pass

    def act(self):
        pass


if __name__ == '__main__':
    sm = StateMachine()

    sm.add_mapping('poi_detected', 'change_antenna', 1)
    sm.add_mapping('obstacle_detected', 'stop_motors', 1)
    sm.add_mapping('obstacle_detected', 'go_back', 2)
    sm.add_mapping('poi_detected', 'rotate', 10)

    print(sm)
