#!/usr/bin/env python
__TODDLER_VERSION__ = "1.0.0"

import time
import numpy
import cv2
import datetime
import numpy as np
from vision import detected_colored_object

# Hardware test code
class Toddler:
    def __init__(self, IO):
        print 'I am a toddler playing in a sandbox'
        self.IO = IO
        self.inp = [0, 0, 0, 0, 0, 0, 0, 0]
        self._poi_color_lower = np.array([0, 0, 48], dtype=np.uint8)
        self._poi_color_upper = np.array([0, 0, 195], dtype=np.uint8)

    def move(self, l, r):
        if not l and not r:
            return [0, 0]
        if l and not r:
            return [100, -100]
        if not l and r:
            return [-100, 100]
        if l and r:
            return [100, 100]


            # This is a callback that will be called repeatedly.

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        mot = [False, False, False]
        motPrev = [False, False, False]
        pos = 180
        while OK():
            time.sleep(0.05)
            self.inp = self.IO.getInputs()
            mot[0] = self.inp[1]
            mot[1] = self.inp[2]
            mot[2] = self.inp[3]
            if mot[0] != motPrev[0] or mot[1] != motPrev[1]:
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
            if mot[2] != motPrev[2] and not mot[2]:
                self.IO.servoDisengage()
            motPrev[0] = mot[0]
            motPrev[1] = mot[1]
            motPrev[2] = mot[2]

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        self.IO.cameraSetResolution('low')
        hasImage = False
        while OK():
            for i in range(0, 5):
                self.IO.cameraGrab()
            img = self.IO.cameraRead()
            if img.__class__ == numpy.ndarray:
                hasImage = True

            if hasImage:
                cv2.imwrite('camera-' + datetime.datetime.now().isoformat() + '.png', img)
                self.IO.imshow('window', img)
                self.IO.setStatus('flash', cnt=2)
                detected, mask = detected_colored_object(img, self._poi_color_lower, self._poi_color_upper)
                cv2.imshow('frame', mask)
                if detected:
                    cv2.imwrite('detected_colored_object-' + datetime.datetime.now().isoformat() + '.png', img)

            time.sleep(0.05)
