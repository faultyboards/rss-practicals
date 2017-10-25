#!/usr/bin/env python

import control

__TODDLER_VERSION__ = "Best One"

from motors import Motors
from sensors import Sensors
from motion import Motion


class Toddler:
    def __init__(self, IO):
        print('I am a toddler playing in a sandbox')
        self.IO = IO
        self.sensors = Sensors(self.IO)
        self.motors = Motors(self.IO)
        self.motion = Motion(self.IO)
        self._poi_position = []
        self._robot_position = []

        # self.mode = 'navigation'
        self.mode = 'poi_search'

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        while OK():
            if self.mode == 'navigation':
                # Go until obstacle is detected
                dist_travelled = self.motion(_simple_stop_condition_callback, 'callback')
                if self.sensors.get_light('front') == poi:
                    # Move forward until get on the poi (15is cm?)
                    self.motion.move(0.15)
                elif self.sensors.get_whisker():
                    # We bumped into an obstacle - back off
                    self.motion.move(-0.5 * dist_travelled)
                    # Then check where is there more space
                    if self.sensors.get_ir['left'] < self.sensors.get_ir['right']:
                        self.motion.turn(90, 'degrees')
                    else:
                        self.motion.turn(-90, 'degrees')
                    # Then continue with your travel
            elif self.mode == 'poi_search':
                # Go until obstacle is detected
                dist_travelled = self.motion(_simple_stop_condition_callback, 'callback')
                if self.sensors.get_light('front') == poi:
                    # Move forward until get on the poi (15is cm?)
                    self.motion.move(0.15)
                    print('Finished!')
                    while True:
                        pass
                elif self.sensors.get_whisker():
                    # We bumped into an obstacle - back to the start location
                    self.motion.move(-1 * dist_travelled)
                elif dist_travelled > 0.4:
                    self.motion.turn(15, 'degrees')
                    # Neither of them so continue the search but at other angle


    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            pass

    def _simple_stop_condition_callback(distance):
        distance_allowed = 0.4
        return distance > distance_allowed or \
               self.sensors.get_light('front') == 'poi' or \
               self.sensors.get_whisker()