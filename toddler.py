#!/usr/bin/env python
from motion import Motion
from motors import Motors
from sensors import Sensors
from wallwalker import Wallwalker

import numpy as np

__TODDLER_VERSION__ = "Best One"


class Toddler:
    def __init__(self, IO):
        print('I am a toddler playing in a sandbox')
        self.IO = IO
        self.sensors = Sensors(self.IO)
        self.motion = Motion(self.IO)
        self.motors = Motors(self.IO)
        self.wallwalker = Wallwalker(self.sensors, self.motion)

        self.SAMPL = 100
        self.poi_size = 0.2  # TODO

        self.readings_idx = 0
        self.readings = np.empty(self.SAMPL)

        self.segs_with_poi = []

    def cb_poi_light():
        '''
        Callback for motion module stopping at a poi or obstacle.
        '''
        return False # TODO

    def poi_go(self, poi_position):
        '''
        Head towards poi using vision information.
        '''
        heading = self.wallwalker.get_xy_th_estimate()[2]
        angle_turned = 0
        disp_to_angle_mul = 1  # TODO
        epsilon = 1  # TODO
        dist_from_centerline = poi_position[0] - self.vision.centerline
        while dist_from_centerline > epsilon:
            angle_turned += self.motion.turn(
                -dist_from_centerline * disp_to_angle_mul)
            poi_position = self.vision.get_poi_position()
            dist_from_centerline = poi_position[0] - self.vision.centerline

        heading += angle_turned
        # Heading towards POI -> Now walk until you are touching it
        travel = self.motion.move(cb_poi_light, 'callback')
        # enter poi itself
        travel += self.motion.move(self.poi_size)
        return travel, heading

    def get_pos_est(self, travel, heading):
        '''
        Calculate the estimated arena x,y,theta position given what we walker
        so far
        '''
        # self.wallwalker.get_xy_th_estimate()
        pass

    def poi_service(self, pos_est):
        '''
        Align antenna given current estimated position
        '''
        pass

    def poi_return(self, poi_travel):
        '''
        Return to the last place we knew on the track.
        '''
        pass

    def poi_decide_what_to_do(self, poi_position):
        if self.wallwalker.current_segment not in self.segs_with_poi:
            if poi_position is None:
                print("No poi detected in the segment yet!")
                return
            else:
                # We just lost poi -> need to backtract
                print("Lost poi -> backtrack until found")
                self.wallwalker.unstep()
                poi_travel, poi_heading = self.poi_go(poi_position)
                pos_est = self.get_pos_est(poi_travel, poi_heading)
                self.poi_service(pos_est)
                self.poi_return(poi_travel)
        else:
            # We've seen the poi already in this segment and are still seeing
            # it
            self.last_poi_position = poi_position

            # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        while OK():
            if (self.wallwalker.current_segment >=
                    len(self.wallwalker._segment_info)):
                print('Finished')
                while True:
                    pass
            poi_position = self.vision.get_poi_position()
            self.poi_decide_what_to_do(poi_position)
            self.wallwalker.step()

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            pass
