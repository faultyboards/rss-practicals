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
