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
                dist_travelled = self.motion.move(Toddler._simple_stop_condition_callback, 'callback')
                if reason == 'poi':
                    # Move forward until get on the poi (15is cm?)
                    self.motion.move(0.15)
                elif reason == 'whisker':
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
                print('Going forward')
                dist_travelled, reason = self.motion.move(Toddler._simple_stop_condition_callback, 'callback')
                if reason == 'whisker':
                    # We bumped into an obstacle - back to the start location
                    print('OBSTACLE')
                    self.motion.move(-1 * dist_travelled)
                    print('Turning for next try')
                    self.motion.turn(15, 'degrees')
                elif reason == 'distance':
                    print('backing off')
                    self.motion.move(-1 * dist_travelled)
                    print('Turning for next try')
                    self.motion.turn(15, 'degrees')
                    # Neither of them so continue the search but at other angle
                # if self.sensors.get_light('front') == 'poi':
                elif reason == 'poi':
                    # Must be POI
                    # Move forward until get on the poi (15is cm?)
                    print('POI detected')
                    self.motion.move(0.15)
                    print('Finished!')
                    while True:
                        pass
                else:
                    print('???')


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