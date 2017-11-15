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
        self.curr_small_ang_dev = 0
        self.old_wall_dist = None
        self.poi_size = 0.2  # TODO
        self.transmission_time = 10  # TODO

        self.dist_since_being_accurate = 0

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

    def correct_heading(self, dist_last_travelled, desired_wall_dist):
        '''
        Turn to attempt following the wall at roughly the same distance.
        '''
        curr_wall_meas = self.sensors.get_ir('right')
        if self.old_wall_dist is None:
            self.old_wall_dist = curr_wall_meas

        curr_wall_dist = curr_wall_meas
        minimum_dist_to_travel = 0.1

        if dist_last_travelled > minimum_dist_to_travel:
            self.curr_small_ang_dev = np.arcsin((curr_wall_dist - self.old_wall_dist) / dist_last_travelled)
            correction_heading = np.arctan((curr_wall_dist - desired_wall_dist) / self.wallwalker.get_step_size())
            print('last_dist_travelled {}'.format(dist_last_travelled))
            print('Distance from the wall {} / {} / {}'.format(self.old_wall_dist, curr_wall_dist, desired_wall_dist))
            print('Correcting heading {} / {} / {}'.format(self.curr_small_ang_dev, correction_heading, self.curr_small_ang_dev + correction_heading))
            self.motion.turn((self.curr_small_ang_dev + correction_heading) * 0.9)
        else:
            print('Travelled for a shor distance -> Dont correct heading fully')
        self.old_wall_dist = curr_wall_dist

    def correct_heading2(self, dist_last_travelled, desired_wall_dist):
        '''
        Turn to attempt following the wall at roughly the same distance.
        2nd method
        '''
        curr_wall_meas = self.sensors.get_ir('right', method='accurate')
        allowed_wall_error = 0.1
        wall_error = curr_wall_meas - desired_wall_dist
        self.dist_since_being_accurate += dist_last_travelled
        print('dist: {} / {}, error {}%'.format(curr_wall_meas, desired_wall_dist, np.abs(wall_error) / desired_wall_dist))
        if np.abs(wall_error) / desired_wall_dist >= allowed_wall_error:
            print('correcting heading')
            angl_to_straight = np.arcsin(wall_error / self.dist_since_being_accurate)
            dist_along_wall_since_last_corr = self.dist_since_being_accurate * np.cos(angl_to_straight)
            dist_along_wall_to_go = self.wallwalker.get_segment_len() - dist_along_wall_since_last_corr
            angl_to_correct = np.arctan(wall_error / dist_along_wall_to_go)
            print('angl_to_straight {}\tdist_along_wall_since_last_corr {}\tdist_along_wall_to_go {}\tangl_to_correct {}'.format(angl_to_straight, dist_along_wall_since_last_corr, dist_along_wall_to_go, angl_to_correct))
            self.motion.turn(angl_to_straight + angl_to_correct)
        elif np.abs(wall_error) / desired_wall_dist < allowed_wall_error / 10:
            self.dist_since_being_accurate = 0

    def correct_heading3(self, dist_last_travelled, desired_wall_dist):
        '''
        Turn to attempt following the wall at roughly the same distance.
        2nd method
        '''
        curr_wall_meas = self.sensors.get_ir('right', method='accurate')
        allowed_wall_error = 0.1
        wall_error = curr_wall_meas - desired_wall_dist
        self.dist_since_being_accurate += dist_last_travelled
        print('dist: {} / {}, error {}%'.format(curr_wall_meas, desired_wall_dist, np.abs(wall_error) / desired_wall_dist))
        if np.abs(wall_error) / desired_wall_dist >= allowed_wall_error:
            print('correcting heading')
            angl_to_straight = np.arcsin(wall_error / self.dist_since_being_accurate)
            dist_along_wall_since_last_corr = self.dist_since_being_accurate * np.cos(angl_to_straight)
            dist_along_wall_to_go = self.wallwalker.get_segment_len() - dist_along_wall_since_last_corr
            angl_to_correct = np.arctan(wall_error / dist_along_wall_to_go)
            print('angl_to_straight {}\tdist_along_wall_since_last_corr {}\tdist_along_wall_to_go {}\tangl_to_correct {}'.format(angl_to_straight, dist_along_wall_since_last_corr, dist_along_wall_to_go, angl_to_correct))
            self.motion.turn(angl_to_straight + angl_to_correct)
        elif np.abs(wall_error) / desired_wall_dist < allowed_wall_error / 10:
            self.dist_since_being_accurate = 0

    def bwait(self):
        # while not self.sensors.get_switch():
        #     pass
        raw_input('Press Enter to proceed')

    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
        self.motion.stop()
        self.bwait()
        # self.motion.move(0.115)
        # self.motion.turn(90, 'degrees')
        print(self.sensors.get_ir('right', method='accurate'))
        self.bwait()
        while OK():
            if self.wallwalker.current_segment >= \
                    len(self.wallwalker._segment_info) or \
                    self.poi_handled >= 3:
                print('Finished')
                self.bwait()
            # Main loop
            # Check for poi
            # poi_position, _ = self.vision.get_poi_location()
            # # Decide what to do
            # self.poi_decide_what_to_do(poi_position)
            # Progress on the track
            dist_travelled = self.wallwalker.step()
            if not self.wallwalker.seg_transition_due:
                self.bwait()
                self.correct_heading(dist_travelled, self.wallwalker.get_targ_wall_dist())
            self.bwait()
            pass

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
        while OK():
            pass
            # print(self.sensors.get_ir('right', raw=True))
            # time.sleep(0.3)
