#!/usr/bin/env python
from vision import detected_colored_object, get_robot_position_from_camera

__TODDLER_VERSION__ = "1.0.0"

import datetime
import time

import cv2
import numpy as np


# Hardware test code
class Toddler:
    def __init__(self, IO):
        print('I am a toddler playing in a sandbox')
        self.IO = IO
        self.inp = [0, 0, 0, 0, 0, 0, 0, 0]
        self._poi_color_lower_range = np.array([169, 100, 100], dtype=np.uint8)
        self._poi_color_upper_range = np.array([189, 255, 255], dtype=np.uint8)
        self._satellite_pose = (np.array([0, 0, 0], dtype=np.float32), np.array([0, 0], dtype=np.float32))
        self._poi_detected = False
        self._position_from_camera = None
        
    def move(self, l, r):
        if not l and not r:
            return [0, 0]
        if l and not r:
            return [100, -100]
        if not l and r:
            return [-100, 100]
        if l and r:
            return [100, 100]

    def _change_antenna_position(self):
        pass

    def sense(self):
        return self.IO.getInputs()

    def plan(self, state):
        pass

    def act(self, mot, mot_prev, pos):

        # detected POI, set up the antenna orientation towards the satellite
        if self._poi_detected:
            self._change_antenna_position()
        else:
            self.inp = self.IO.getInputs()
            mot[1] = self.inp[2]
            mot[0] = self.inp[1]
            mot[2] = self.inp[3]
            if mot[0] != mot_prev[0] or mot[1] != mot_prev[1]:
                speed = self.move(mot[0], mot[1])
                self.IO.setMotors(-speed[0], speed[1])
                if mot[0]:
                    self.IO.setStatus('on')
                else:
                    self.IO.setStatus('off')
                if mot[1]:
                    self.IO.setError('on')
                else:
                    self.IO.setError('off')

            if mot[2]:
                self.IO.servoEngage()
                pos = (pos + 3) % 360
                self.IO.servoSet(abs(pos - 180))
            if mot[2] != mot_prev[2] and not mot[2]:
                self.IO.servoDisengage()
            mot_prev[0] = mot[0]
            mot_prev[1] = mot[1]
            mot_prev[2] = mot[2]

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        mot = [False, False, False]
        motPrev = [False, False, False]
        pos = 180
        while OK():
            time.sleep(0.05)
            self.act(mot, motPrev, pos)

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        self.IO.cameraSetResolution('low')
        hasImage = False
        res = 0
        sw = False
        swPrev = False
        while OK():
            if self.inp[4]:
                for i in range(0, 5):
                    self.IO.cameraGrab()
                img = self.IO.cameraRead()
                self._poi_detected = detected_colored_object(
                    img,
                    self._poi_color_lower_range,
                    self._poi_color_upper_range
                )

                self._position_from_camera = get_robot_position_from_camera(img)

            time.sleep(0.05)
