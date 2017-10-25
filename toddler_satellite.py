#!/usr/bin/env python
from motion import Motion

__TODDLER_VERSION__ = "Best One"

import numpy as np

from sensors import Sensors

SATELLITE_POSITION = [-0.69, 0, 2.95]
ANTENNA_POSITION = [-0.04, -0.09, 0.25]
# ROBOT_POSITION = [0.355, 3.319, 0]
# ROBOT_POSITION = [-0.9, 2.44, 0]
ROBOT_POSITION = [1.4, 1.8, 0]
ROBOT_ORIENTATION = np.pi


def find_satellite(robot_position, robot_orientation, satellite_pos):
    # rotate the antenna to horizontal
    forwardVec = [1, 0]

    antennaPos = np.add(robot_position, ANTENNA_POSITION)

    antenna2Satellite = np.subtract(satellite_pos, antennaPos)

    # normalise
    np.linalg.norm(antenna2Satellite)

    print "antenna2Satellite = ", antenna2Satellite

    # calculate the vector direction of the bot using its angle from global forward
    botDir = [np.add(np.multiply(forwardVec[0], np.cos(robot_orientation)),
                     np.multiply(forwardVec[1], np.sin(robot_orientation))),
              np.add(np.multiply(-forwardVec[0], np.sin(robot_orientation)),
                     np.multiply(forwardVec[1], np.cos(robot_orientation))),
              0]

    # normalise
    np.linalg.norm(botDir)

    print "\nbotdir = ", botDir

    # the x& y values of antenna to satellite
    a2sg = [antenna2Satellite[0], antenna2Satellite[1], 0]
    # a2sg converted into 3d space by adding a 0 in the z axis
    a2s3d = [antenna2Satellite[0], antenna2Satellite[1], 0]

    upVector = [0,0,1]
    normal = np.cross(botDir, a2sg)
    print 'dotDir = ', botDir
    print ' a2sg= ', a2sg
    angle = np.dot(upVector, normal)

    print 'angle = ', angle

    sign = angle / np.abs(angle)

    print 'sign = ', sign


    # find the angle between the bots forward direction and the satellite
    botTurnRad = np.arccos(np.divide(np.dot(botDir, a2sg),
                                     np.multiply(sum(np.multiply(botDir, botDir)) ** 0.5,
                                                 sum(np.multiply(a2sg, a2sg)) ** 0.5)))

    # convert the angle to degrees
    botTurnDeg = (botTurnRad / (2 * np.pi)) * 360

   #botTurnDeg = botTurnDeg * sign


    # find the angle between the plane and the satellite
    antennaRot = np.arccos(np.divide(np.dot(a2s3d, antenna2Satellite),
                                     np.multiply(sum(np.multiply(a2s3d, a2s3d)) ** 0.5,
                                                 sum(antenna2Satellite * antenna2Satellite) ** 0.5)))

    # convert to degrees
    antennaRot = (antennaRot / (2 * np.pi)) * 360

    print antennaRot
    print "\n", botTurnDeg
    return botTurnDeg, antennaRot


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
                #if np.abs(bot_turn_deg) >= 180:
                #    bot_turn_deg = (bot_turn_deg+180)%360
                self.motion.move(0.20)
                self.motion.move(-0.20)
                if ROBOT_ORIENTAT
                self.motion.turn(bot_turn_deg - 90, 'degrees')
                self.motion.set_antenna(antenna_rot, 'degrees')
                goal_reached = True

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            pass
