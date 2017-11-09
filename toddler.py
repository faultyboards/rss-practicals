#!/usr/bin/env python
from motion import Motion
from motors import Motors
from sensors import Sensors
from wallwalker import Wallwalker

import numpy as np

__TODDLER_VERSION__ = "Best One"


class Toddler:
    def __init__(self, IO):
        print('I am a toddler playing in a sandbox')
        self.IO = IO
        self.sensors = Sensors(self.IO)
        self.motion = Motion(self.IO)
        self.motors = Motors(self.IO)
        self.wallwalker = Wallwalker(self.sensors, self.motion)

        self.SAMPL = 100

        self.readings_idx = 0
        self.readings = np.empty(self.SAMPL)

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        while OK():
            # self.wallwalker.step()
            # print(self.wallwalker.distance_along)
            # time.sleep(2)
            reading = self.sensors.get_ir('right')
            self.readings[self.readings_idx] = reading
            self.readings_idx = (self.readings_idx + 1) % self.SAMPL
            print('mean: {}\tsd: {}'.format(np.median(self.readings), self.readings.std()))

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            pass
