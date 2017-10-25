#!/usr/bin/env python

import control

__TODDLER_VERSION__ = "Best One"

from sensors import Sensors
from motion import Motion


class Toddler:
    def __init__(self, IO):
        print('I am a toddler playing in a sandbox')
        self.IO = IO
        self.sensors = Sensors(self.IO)
        self.motion = Motion(self.IO)
        self._poi_position = []
        self._robot_position = []
        self.control = control.Control(
            self.sensors,
            self.motion,
            self._poi_position,
            self._robot_position)

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        state = self.control.init_state()

        while OK():
            new_state = self.control.sense(state)
            self.control.act_navigation(new_state)
            state = new_state


    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            pass

    @staticmethod
    def _simple_stop_condition_callback(sensors, distance):
        distance_allowed = 0.70
        retval = distance > distance_allowed or \
               sensors.get_light('front') == 'poi' or \
               sensors.get_whisker()
        reason = None
        if distance > distance_allowed:
            reason = 'distance'
        elif sensors.get_light('front') == 'poi':
            reason = 'poi'
        elif sensors.get_whisker():
            reason = 'whisker'

        print('{} callback {} {} {}'.format(not retval, distance, sensors.get_light('front'), sensors.get_whisker()))
        return not retval, reason
