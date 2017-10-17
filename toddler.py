#!/usr/bin/env python

__TODDLER_VERSION__ = "Best One"

import time

import cv2
import numpy as np

from sensors import Sensors
from motors import Motors
from vision import detected_colored_object, get_robot_position_from_camera, OpticalFlow


class Toddler:
    def __init__(self, IO):
        print('I am a toddler playing in a sandbox')
        self.IO = IO
        self.sensors = Sensors(self.IO)
        self.motors = Motors(self.IO)

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        while OK():
            pass

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            pass