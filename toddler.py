#!/usr/bin/env python
import datetime

import cv2

from motion import Motion
from vision import Vision

__TODDLER_VERSION__ = "Best One"

from sensors import Sensors
from motors import Motors

class Toddler:
    def __init__(self, IO):
        print('I am a toddler playing in a sandbox')
        self.IO = IO
        self.sensors = Sensors(self.IO)
        self.motion = Motion(self.IO)
        self.motors = Motors(self.IO)
        self.vision = Vision(self.IO)

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        while OK():
            pass

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            img = self.vision.grab_image()
            self.IO.imshow("image", img)
            cv2.imwrite('camera-' + datetime.datetime.now().isoformat() + '.png', img)
            poi, poi_contours = self.vision.get_poi_location(img)

    self.IO.imshow("contours", poi_contours)
    if poi:
        print("poi recognized: {}".format(poi))


self.IO.imshow("contours", poi_contours)
