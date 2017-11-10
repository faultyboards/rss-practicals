#!/usr/bin/env python

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
            pass
