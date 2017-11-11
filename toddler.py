#!/usr/bin/env python

from motion import Motion
from motors import Motors
from sensors import Sensors
from vision import Vision
from wallwalker import Wallwalker

import numpy as np
import time

__TODDLER_VERSION__ = "Best One"


class Toddler:
    def __init__(self, IO):
        print('I am a toddler playing in a sandbox')
        self.IO = IO
        self.sensors = Sensors(self.IO)
        self.motors = Motors(self.IO)
        self.motion = Motion(self.IO, self.sensors, self.motors)
        self.wallwalker = Wallwalker(self.sensors, self.motion)
        self.vision = Vision(self.IO)

        self.satellite_pos = np.array([-0.69, 0, 2.95])
        self.antenna_pos = np.array([-0.04, -0.09, 0.25])
        self.poi_size = 0.2  # TODO
        self.transmission_time = 10  # TODO

        self.segs_with_poi = []
        self.poi_handled = 0

    def cb_poi_light(sensors, distance_travelled):
        '''
        Callback for motion module stopping at a poi or obstacle.
        '''
        if sensors.get_light['front'] == 'poi':
            return False, 'poi'
        elif sensors.get_whisker():
            return False, 'obstacle'
        else:
            return True

    def poi_go(self, poi_position):
        '''
        Head towards poi using vision information.
        '''
        angle_turned = 0
        disp_to_angle_mul = 1  # TODO
        epsilon = 1  # TODO
        dist_from_centerline = poi_position[0] - self.vision.centerline
        while dist_from_centerline > epsilon:
            angle_turned += self.motion.turn(
                -dist_from_centerline * disp_to_angle_mul)
            poi_position, _ = self.vision.get_poi_location()
            dist_from_centerline = poi_position[0] - self.vision.centerline

        # Heading towards POI -> Now walk until you are touching it
        travel, stop_cond = self.motion.move(self.cb_poi_light, 'callback')
        if stop_cond == 'poi':
            # enter poi itself
            travel += self.motion.move(self.poi_size)

        return travel, angle_turned, stop_cond == 'poi'

    def get_pos_est(self, travel, angle_turned):
        '''
        Calculate the estimated arena x,y,theta position given what we walker
        so far
        '''
        track_position = self.wallwalker.get_xy_th_estimate()
        heading = track_position[2] + angle_turned
        current_position = track_position + travel * np.array([np.cos(heading),
                                                               np.sin(heading),
                                                               0])
        return current_position

    def poi_service(self, pos_est):
        '''
        Align antenna given current estimated position
        '''
        # Calculate the angles to turn
        robot_pos_3d = pos_est.copy()
        robot_orientation = robot_pos_3d[2]
        robot_pos_3d[2] = 0
        r = self.satellite_pos - robot_pos_3d
        r_f = np.concatenate([r[:2], np.zeros(1)])

        ant_angl = (np.pi / 2 -
                    np.arccos(np.dot(r, np.array([0, 0, 1])) /
                              np.linalg.norm(r))) % (2 * np.pi)
        bot_turn_deg = ((1 if r_f[1] < 0 else -1) *
                        np.arccos(np.dot(r_f, np.array([1, 0, 0])) /
                                  np.linalg.norm(r_f)) +
                        np.pi / 2 -
                        robot_orientation) % (2 * np.pi)

        if bot_turn_deg < -np.pi / 2:
            bot_turn_deg += np.pi
        elif bot_turn_deg > np.pi / 2:
            bot_turn_deg -= np.pi
        else:
            ant_angl = np.pi - ant_angl

        angle_turned = self.motion.turn(bot_turn_deg)
        self.motion.set_antenna(ant_angl)
        time.sleep(self.transmission_time)
        self.motion.turn(-angle_turned)

    def poi_return(self, travel, angle_turned):
        '''
        Return to the last place we knew on the track.
        '''
        self.motion.move(-travel)
        self.motion.turn(-angle_turned)

    def poi_decide_what_to_do(self, poi_position):
        if self.wallwalker.current_segment not in self.segs_with_poi:
            if poi_position is None:
                print("No poi detected in the segment yet!")
                return
            else:
                # Seeing poi for the first time in this segment
                self.segs_with_poi.append(self.wallwalker.current_segment)
        else:
            if poi_position is None:
                # We just lost poi -> need to backtrack
                print("Lost poi -> backtrack until found")
                self.wallwalker.unstep()
                # And now aim to go for the poi
                poi_travel, angle_turned, success = self.poi_go(poi_position)
                # Only do the antenna alignment stuff if we actually entered a
                # poi
                if success:
                    pos_est = self.get_pos_est(poi_travel, angle_turned)
                    self.poi_service(pos_est)
                self.poi_return(poi_travel, angle_turned)
                self.poi_handled += 1
            else:
                # Seeing the poi again -> do nothing
                pass
            # We've seen the poi already in this segment and are still seeing
            # it
            self.last_poi_position = poi_position

            # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        while OK():
            if self.wallwalker.current_segment >= \
                    len(self.wallwalker._segment_info) or \
                    self.poi_handled >= 3:
                print('Finished')
                while True:
                    pass

            # Main loop
            # Check for poi
            poi_position, _ = self.vision.get_poi_location()
            # Decide what to do
            self.poi_decide_what_to_do(poi_position)
            # Progress on the track
            self.wallwalker.step()

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            pass
