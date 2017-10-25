#!/usr/bin/env python
from motion import Motion

__TODDLER_VERSION__ = "Best One"

import numpy as np

from sensors import Sensors

SATELLITE_POSITION = [-0.69, 0, 2.95]
ANTENNA_POSITION = [-0.04, -0.09, 0.25]
# ROBOT_POSITION = [0.355, 3.319, 0]
# ROBOT_POSITION = [-0.9, 2.44, 0]
ROBOT_POSITION = [-0.69, 0, 0]

# ROBOT_POSITION = [1.4, 1.8, 0]
ROBOT_ORIENTATION = 0


def find_satellite(robot_position, robot_orientation, satellite_pos):
    p = np.array(robot_position)
    s = np.array(satellite_pos)
    r = s - p
    r_f = np.concatenate([r[:2], np.zeros(1)])

    ant_angl = np.pi/2 - np.arccos(np.dot(r,np.array([0,0,1]))/np.linalg.norm(r))
    bot_turn_deg = robot_orientation + np.arccos(np.dot(r_f,np.array([1,0,0]))/np.linalg.norm(r_f)) + np.pi/2
    
    if bot_turn_deg < -np.pi/2:
        bot_turn_deg += np.pi
        ant_angl = np.pi - ant_angl
    elif bot_turn_deg > np.pi/2:
        bot_turn_deg -= np.pi
        ant_angl = np.pi - ant_angl

    return bot_turn_deg, ant_angl

class Toddler:
    def __init__(self, IO):
        print('I am a toddler playing in a sandbox')
        self.IO = IO
        self.sensors = Sensors(self.IO)
        self.motion = Motion(self.IO)

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        goal_reached = False
        while OK():
            if not goal_reached:
                bot_turn_deg, antenna_rot = find_satellite(ROBOT_POSITION, ROBOT_ORIENTATION, SATELLITE_POSITION)
                print bot_turn_deg * 180 / np.pi, antenna_rot  * 180 / np.pi

                self.motion.move(0.30)
                self.motion.move(-0.30)
                self.motion.turn(bot_turn_deg)
                self.motion.set_antenna(antenna_rot)
                goal_reached = True

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            pass
