import numpy as np


class Sensors:
    def __init__(self, IO):
        self.IO = IO

        #  define ports for sensors
        self.port = {}
        # digital sensors
        self.port['whisker'] = {'left': 0, 'right': 1}
        self.port['switch'] = [2, 3, 4, 5]
        self.port['hall'] = -1

        # analogue sensors
        self.port['sonar'] = 0
        self.port['light'] = {'front': 1, 'rear': 2, 'left': 3, 'right': 4}
        self.port['ir'] = {'left': 5, 'right': 6}

        # define thresholds / multipliers for analogue sensors
        # Rear and side light sensor readings are NOT reliable
        self.whskr_on_state = {'left': 1, 'right': 1}
        self.light_threshold = {'front': 120,
                                'rear': 255,
                                'left': 45,
                                'right': 25}
        self.ir_inv_dist_fnctn = np.poly1d([-3.38526873e+10, 1.43660335e+09, -1.87399879e+07, 1.04674902e+05, -1.85540076e+02, 1.86315734e-01])
        self.sonar_dist_mltpl = [.01, 0.076]

        # for storing most recent sensor readings
        self.analogue_readings = self.IO.getSensors()
        self.digital_readings = self.IO.getInputs()

        # Fix sonar resolution
        self.IO._interfaceKit.setSensorChangeTrigger(self.port['sonar'], 1)

        self.default_to_raw = False

    def update_readings(self, type='both'):
        if type == 'analogue' or type == 'both':
            self.analogue_readings = self.IO.getSensors()
        if type == 'digital' or type == 'both':
            self.digital_readings = self.IO.getInputs()

    def get_whisker(self, side='either'):
        if side == 'either':
            return (self.digital_readings[self.port['whisker']['left']] or
                    self.digital_readings[self.port['whisker']['right']])
        elif side == 'both':
            return (self.digital_readings[self.port['whisker']['left']] and
                    self.digital_readings[self.port['whisker']['right']])
        else:
            return self.digital_readings[self.port['whisker'][side]]

    def get_switch(self, switch_no='either'):
        if switch_no == 'either':
            return sum([self.digital_readings[x] for x in self.port['switch']])
        else:
            return self.digital_readings[self.port['switch'][switch_no]]

    def get_sonar(self, raw=False):
        if not raw:
            reading = self.analogue_readings[self.port['sonar']]
            return (self.sonar_dist_mltpl[0] +
                    self.sonar_dist_mltpl[1] * reading)
        else:
            return self.analogue_readings[self.port['sonar']]

    def get_ir(self, sensor_loc, raw=False, method='fast'):
        if method == 'fast':
            samples_no = 500
        elif method == 'accurate':
            samples_no = 10000

        if sensor_loc =='both':
            readings = np.empty((samples_no, 2))
        else:
            readings = np.empty(samples_no)

        for i in range(samples_no):
            self.update_readings(type='analogue')
            # sensor_loc = 'left' if sensor_loc == 'front' else sensor_loc
            # sensor_loc = 'right' if sensor_loc == 'back' else sensor_loc
            if sensor_loc =='both':
                readings[i,0] = self.analogue_readings[self.port['ir']['right']]
                readings[i,1] = self.analogue_readings[self.port['ir']['left']]
            else:
                readings[i] = self.analogue_readings[self.port['ir'][sensor_loc]]

        if sensor_loc =='both':
            r_med = np.median(readings, axis=0)
            r_std = np.expand_dims(readings.std(axis=0), 1).T
            readings_no_outliers = readings[(readings - r_med <= 2 * r_std).sum(axis=1) > 0, :]
            reading = np.mean(readings_no_outliers, axis=0)
        else:
            r_med = np.median(readings)
            r_std = readings.std()
            readings_no_outliers = [reading for reading in readings
                                    if np.abs(r_med - reading) <= 2 * r_std]
            reading = np.mean(readings_no_outliers)

        if not raw:
            return self.ir_inv_dist_fnctn(1./reading)
        else:
            return self.analogue_readings[self.port['ir'][sensor_loc]]

    def get_light(self, sensor_loc, raw=False):
        if not raw:
            return ('poi' if
                    self.analogue_readings[self.port['light'][sensor_loc]] >
                    self.light_threshold[sensor_loc]
                    else 'floor')
        else:
            return self.analogue_readings[self.port['light'][sensor_loc]]

    def get_hall(self):
        return self.digital_readings[self.port['hall']]
