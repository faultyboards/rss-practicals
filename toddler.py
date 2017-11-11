#!/usr/bin/env python
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
        i = 0
        while OK():

            if self.sensors.get_switch(0):
                img = self.vision.grab_image(5)
                self.IO.imshow(img)
                cv2.imwrite('camera-' + str(i) + '.png', img)
                poi_location, moments = self.vision.get_poi_location(img)

                if poi_location:
                    print("Correctly recognized POI!")
                    print(poi_location)
                else:
                    print("unable able to get POI")
