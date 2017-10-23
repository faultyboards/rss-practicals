#!/usr/bin/env python

import control

__TODDLER_VERSION__ = "Best One"

from motors import Motors
from sensors import Sensors


class Toddler:
    def __init__(self, IO, poi_position=None, robot_position=None):
        print('I am a toddler playing in a sandbox')
        self.IO = IO
        self.sensors = Sensors(self.IO)
        self.motors = Motors(self.IO)
        self.control = control.Control(self.sensors, self.motors)
        self._poi_position = poi_position
        self._robot_position = robot_position
        self._initial_state = {}

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        state = self._initial_state

        while OK():
            new_state = self.control.sense(state)
            self.control.act(new_state)
            state = new_state

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            pass
