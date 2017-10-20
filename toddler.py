#!/usr/bin/env python

__TODDLER_VERSION__ = "Best One"

import time
import sys

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
        log_file = 'sensors.log'
        script = [{'type': 'light', 'meas1': ['floor','home','poi'], 'meas2': [''], 'locations':['front', 'rear', 'left', 'right']},
                  {'type': 'sonar', 'meas1': ['straight', '40deg'], 'meas2': ['20cm', '50cm', '100cm', '150cm'], 'locations': None},
                  {'type': 'ir', 'meas1': ['straight', '40deg'], 'meas2': ['5cm', '10cm', '20cm', '50cm', '80cm', '100cm', '120cm'], 'locations': ['left', 'right']}]

        self.sensors.default_to_raw = True

        with open(log_file, 'w') as f:
            for sensor in script:
                self.tee('\nEvaluating {}:\n'.format(sensor['type']))
                callfunction = eval('self.sensors.get_{}'.format(sensor['type']))
                readings = []

                for meas1 in sensor['meas1']:
                     for meas2 in sensor['meas2']:
                        self.tee('{} {}: '.format(meas1, meas2))
        
                        for reading_no in range(3):
                            if sensor['locations'] is not None:
                                readings[reading_no] = callfunction(sensor['locations'])
                            else:
                                readings[reading_no] = callfunction()
        
                        self.tee('{} {} {}\n'.format(readings[0], readings[1], readings[2]))
        print('Finished!')         
        while OK():
            pass

    def tee(stringg, f):
        sys.stdout.write(stringg)
        f.write(stringg)
